#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"


#define WIFI_SSID           "MARICOTA"
#define WIFI_PASSWORD       "Vitoria@01"

#define MQTT_HOST           "thingsboard.cloud"
#define MQTT_BROKER_PORT    1883
#define MQTT_TOKEN          "U09KROFvgI7kssvGVWBr"
#define MQTT_TOPIC_TELEM    "v1/devices/me/telemetry"

#define BTN_A_PIN           5
#define BTN_B_PIN           6

#define LED_G_PIN           11
#define LED_B_PIN           12
#define LED_R_PIN           13

#define I2C0_SDA_PIN         0
#define I2C0_SCL_PIN         1
#define I2C1_SDA_PIN         2
#define I2C1_SCL_PIN         3

#define BH1750_ADDR          0x23
#define AHT10_ADDR           0x38

#define WIFI_CONNECTED_BIT  (1u << 0)
#define MQTT_CONNECTED_BIT  (1u << 1)

typedef struct {
    float temperature_c;
    uint16_t light_level;
    uint8_t btn_a;
    uint8_t btn_b;
} sensor_data_t;

static QueueHandle_t sensor_queue;
static EventGroupHandle_t wifi_event_group;
static mqtt_client_t *mqtt_client;
static TaskHandle_t wifi_task_handle;
static TaskHandle_t sensor_task_handle;
static TaskHandle_t mqtt_task_handle;
static SemaphoreHandle_t xPrintfMutex;
static SemaphoreHandle_t xI2cMutex;

typedef struct {
    SemaphoreHandle_t done;
    ip_addr_t *out_ip;
    err_t result;
} dns_lookup_t;

static void thread_safe_printf(const char *format, ...) {
    BaseType_t scheduler_running = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
    if (scheduler_running && xPrintfMutex) {
        xSemaphoreTake(xPrintfMutex, portMAX_DELAY);
    }

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    if (scheduler_running && xPrintfMutex) {
        xSemaphoreGive(xPrintfMutex);
    }
}

static void set_led_rgb(bool r, bool g, bool b) {
    gpio_put(LED_R_PIN, r ? 1 : 0);
    gpio_put(LED_G_PIN, g ? 1 : 0);
    gpio_put(LED_B_PIN, b ? 1 : 0);
}

static void gpio_init_board(void) {
    // Configura botoes e LED RGB da BitDogLab.
    gpio_init(BTN_A_PIN);
    gpio_set_dir(BTN_A_PIN, GPIO_IN);
    gpio_pull_up(BTN_A_PIN);

    gpio_init(BTN_B_PIN);
    gpio_set_dir(BTN_B_PIN, GPIO_IN);
    gpio_pull_up(BTN_B_PIN);

    gpio_init(LED_R_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_init(LED_G_PIN);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);
    gpio_init(LED_B_PIN);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);

    set_led_rgb(true, false, false);
}

static void i2c_init_board(void) {
    // Configura I2C0 (BH1750) e I2C1 (AHT10). Ajuste os pinos se necessario.
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_PIN);
    gpio_pull_up(I2C0_SCL_PIN);

    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_PIN);
    gpio_pull_up(I2C1_SCL_PIN);
}

static bool bh1750_init(void) {
    uint8_t cmd_power_on = 0x01;
    uint8_t cmd_cont_h = 0x10;
    if (xI2cMutex) {
        xSemaphoreTake(xI2cMutex, portMAX_DELAY);
    }
    int ret = i2c_write_blocking(i2c0, BH1750_ADDR, &cmd_power_on, 1, false);
    if (ret < 0) {
        if (xI2cMutex) {
            xSemaphoreGive(xI2cMutex);
        }
        return false;
    }
    ret = i2c_write_blocking(i2c0, BH1750_ADDR, &cmd_cont_h, 1, false);
    if (xI2cMutex) {
        xSemaphoreGive(xI2cMutex);
    }
    return ret >= 0;
}

static bool bh1750_read_lux(uint16_t *lux) {
    uint8_t data[2];
    if (xI2cMutex) {
        xSemaphoreTake(xI2cMutex, portMAX_DELAY);
    }
    int ret = i2c_read_blocking(i2c0, BH1750_ADDR, data, 2, false);
    if (ret < 0) {
        if (xI2cMutex) {
            xSemaphoreGive(xI2cMutex);
        }
        return false;
    }
    if (xI2cMutex) {
        xSemaphoreGive(xI2cMutex);
    }
    uint16_t raw = (uint16_t)((data[0] << 8) | data[1]);
    float lux_f = raw / 1.2f;
    if (lux_f < 0.0f) {
        lux_f = 0.0f;
    }
    if (lux_f > 65535.0f) {
        lux_f = 65535.0f;
    }
    *lux = (uint16_t)lux_f;
    return true;
}

static bool aht10_init(void) {
    uint8_t cmd[3] = {0xE1, 0x08, 0x00};
    if (xI2cMutex) {
        xSemaphoreTake(xI2cMutex, portMAX_DELAY);
    }
    int ret = i2c_write_blocking(i2c1, AHT10_ADDR, cmd, 3, false);
    if (xI2cMutex) {
        xSemaphoreGive(xI2cMutex);
    }
    return ret >= 0;
}

static bool aht10_read_temp(float *temp_c) {
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    uint8_t data[6];

    if (xI2cMutex) {
        xSemaphoreTake(xI2cMutex, portMAX_DELAY);
    }
    int ret = i2c_write_blocking(i2c1, AHT10_ADDR, cmd, 3, false);
    if (ret < 0) {
        if (xI2cMutex) {
            xSemaphoreGive(xI2cMutex);
        }
        return false;
    }
    if (xI2cMutex) {
        xSemaphoreGive(xI2cMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(80));
    if (xI2cMutex) {
        xSemaphoreTake(xI2cMutex, portMAX_DELAY);
    }
    ret = i2c_read_blocking(i2c1, AHT10_ADDR, data, 6, false);
    if (ret < 0) {
        if (xI2cMutex) {
            xSemaphoreGive(xI2cMutex);
        }
        return false;
    }
    if (xI2cMutex) {
        xSemaphoreGive(xI2cMutex);
    }

    uint32_t raw_temp = ((uint32_t)(data[3] & 0x0F) << 16) |
                        ((uint32_t)data[4] << 8) |
                        data[5];

    *temp_c = ((raw_temp * 200.0f) / 1048576.0f) - 50.0f;
    return true;
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    (void) client;
    (void) arg;

    if (status == MQTT_CONNECT_ACCEPTED) {
        xEventGroupSetBits(wifi_event_group, MQTT_CONNECTED_BIT);
        thread_safe_printf("MQTT conectado.\n");
    } else {
        xEventGroupClearBits(wifi_event_group, MQTT_CONNECTED_BIT);
        thread_safe_printf("MQTT falhou, status=%d\n", status);
    }
}

static void dns_found_cb(const char *name, const ip_addr_t *ipaddr, void *arg) {
    (void) name;
    dns_lookup_t *ctx = (dns_lookup_t *)arg;

    if (ctx) {
        if (ipaddr) {
            *(ctx->out_ip) = *ipaddr;
            ctx->result = ERR_OK;
        } else {
            ctx->result = ERR_TIMEOUT;
        }
        if (ctx->done) {
            xSemaphoreGive(ctx->done);
        }
    }
}

static bool resolve_hostname(const char *host, ip_addr_t *out_ip, TickType_t timeout_ticks) {
    dns_lookup_t ctx = {
        .done = xSemaphoreCreateBinary(),
        .out_ip = out_ip,
        .result = ERR_ARG
    };

    if (!ctx.done) {
        return false;
    }

    cyw43_arch_lwip_begin();
    err_t err = dns_gethostbyname(host, out_ip, dns_found_cb, &ctx);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        vSemaphoreDelete(ctx.done);
        return true;
    }

    if (err != ERR_INPROGRESS) {
        vSemaphoreDelete(ctx.done);
        return false;
    }

    if (xSemaphoreTake(ctx.done, timeout_ticks) == pdTRUE && ctx.result == ERR_OK) {
        vSemaphoreDelete(ctx.done);
        return true;
    }

    vSemaphoreDelete(ctx.done);
    return false;
}

static bool mqtt_connect(void) {
    ip_addr_t broker_ip;
    err_t err;
    struct mqtt_connect_client_info_t ci;

    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico_w_bdt";
    ci.client_user = MQTT_TOKEN;
    ci.client_pass = NULL;
    ci.keep_alive = 60;

    if (!resolve_hostname(MQTT_HOST, &broker_ip, pdMS_TO_TICKS(10000))) {
        thread_safe_printf("DNS falhou para %s\n", MQTT_HOST);
        return false;
    }

    cyw43_arch_lwip_begin();
    err = mqtt_client_connect(mqtt_client, &broker_ip, MQTT_BROKER_PORT,
                              mqtt_connection_cb, NULL, &ci);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        thread_safe_printf("MQTT connect err=%d\n", err);
        return false;
    }

    return true;
}

static void wifi_task(void *param) {
    (void) param;
    thread_safe_printf("Tarefa Wi-Fi rodando (Hardware ja iniciado no Main).\n");

    for (;;) {
        // Mantem reconexao automatica ao Wi-Fi.
        thread_safe_printf("Conectando ao Wi-Fi...\n");
        set_led_rgb(true, false, false);

        int ret = cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID,
            WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK,
            30000
        );

        if (ret == 0) {
            thread_safe_printf("Wi-Fi conectado.\n");
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
            set_led_rgb(false, true, false);

            while (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP) {
                vTaskDelay(pdMS_TO_TICKS(2000));
            }

            thread_safe_printf("Wi-Fi desconectado.\n");
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        } else {
            thread_safe_printf("Falha ao conectar Wi-Fi, ret=%d\n", ret);
        }

        set_led_rgb(true, false, false);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void sensor_task(void *param) {
    (void) param;
    bool bh1750_ok = bh1750_init();
    bool aht10_ok = aht10_init();
    uint8_t last_btn_a = 0;
    uint8_t last_btn_b = 0;
    float last_temp_c = 0.0f;
    uint16_t last_lux = 0;

    if (!bh1750_ok) {
        thread_safe_printf("BH1750 nao respondeu na inicializacao.\n");
    }
    if (!aht10_ok) {
        thread_safe_printf("AHT10 nao respondeu na inicializacao.\n");
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
    thread_safe_printf("Tarefa Sensores rodando.\n");

    for (;;) {
        sensor_data_t data;
        data.btn_a = gpio_get(BTN_A_PIN) ? 0 : 1;
        data.btn_b = gpio_get(BTN_B_PIN) ? 0 : 1;

        bool edge_a = data.btn_a && !last_btn_a;
        bool edge_b = data.btn_b && !last_btn_b;

        if (edge_a || edge_b) {
            if (!bh1750_read_lux(&data.light_level)) {
                data.light_level = last_lux;
            }

            if (!aht10_read_temp(&data.temperature_c)) {
                data.temperature_c = last_temp_c;
            }

            last_temp_c = data.temperature_c;
            last_lux = data.light_level;

            xQueueOverwrite(sensor_queue, &data);

            if (edge_a) {
                thread_safe_printf("FELIZ: temp=%.2fC lux=%u\n",
                                  last_temp_c,
                                  last_lux);
            }

            if (edge_b) {
                thread_safe_printf("TRISTE: temp=%.2fC lux=%u\n",
                                  last_temp_c,
                                  last_lux);
            }
        }

        last_btn_a = data.btn_a;
        last_btn_b = data.btn_b;

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void mqtt_task(void *param) {
    (void) param;
    char payload[128];

    mqtt_client = mqtt_client_new();
    if (!mqtt_client) {
        thread_safe_printf("Falha ao criar cliente MQTT.\n");
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
    thread_safe_printf("Tarefa MQTT rodando.\n");

    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT,
            pdFALSE,
            pdTRUE,
            pdMS_TO_TICKS(1000)
        );

        if ((bits & WIFI_CONNECTED_BIT) == 0) {
            if ((bits & MQTT_CONNECTED_BIT) != 0) {
                cyw43_arch_lwip_begin();
                mqtt_disconnect(mqtt_client);
                cyw43_arch_lwip_end();
                xEventGroupClearBits(wifi_event_group, MQTT_CONNECTED_BIT);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if ((bits & MQTT_CONNECTED_BIT) == 0) {
            mqtt_connect();
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        sensor_data_t data;
        if (xQueueReceive(sensor_queue, &data, pdMS_TO_TICKS(2000)) == pdTRUE) {
            int len = snprintf(payload, sizeof(payload),
                               "{\"temperature\": %.2f, \"light_level\": %u, \"btn_a\": %u, \"btn_b\": %u}",
                               data.temperature_c,
                               data.light_level,
                               data.btn_a,
                               data.btn_b);

            if (len > 0) {
                cyw43_arch_lwip_begin();
                err_t err = mqtt_publish(mqtt_client, MQTT_TOPIC_TELEM,
                                         payload, len, 0, 0, NULL, NULL);
                cyw43_arch_lwip_end();

                if (err != ERR_OK) {
                    thread_safe_printf("MQTT publish err=%d\n", err);
                    xEventGroupClearBits(wifi_event_group, MQTT_CONNECTED_BIT);
                } else {
                    thread_safe_printf("MQTT enviado: %s\n", payload);
                }
            }
        }
    }
}


int main(void) {
    stdio_init_all();

    gpio_init_board();
    i2c_init_board();

    printf("Iniciando Wi-Fi no Core 0...\n");
    if (cyw43_arch_init() != 0) {
        printf("Falha ao iniciar Wi-Fi (Hardware Error)\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();
    printf("Wi-Fi Hardware Iniciado com Sucesso!\n");

    set_led_rgb(true, false, false);

    xPrintfMutex = xSemaphoreCreateMutex();
    xI2cMutex = xSemaphoreCreateMutex();

    printf("Esperando USB...\n");
    for (int i = 0; i < 50; i++) {
        if (stdio_usb_connected()) {
            printf("USB Conectado!\n");
            break;
        }
        sleep_ms(100);
    }

    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    set_led_rgb(false, false, true);

    if (stdio_usb_connected()) {
        printf("\n\n--- INICIANDO SISTEMA BITDOGLAB ---\n");
        printf("Hardware inicializado. LED deve estar AZUL.\n");
    }

    sensor_queue = xQueueCreate(1, sizeof(sensor_data_t));
    wifi_event_group = xEventGroupCreate();

    BaseType_t status_wifi = xTaskCreate(wifi_task, "wifi_task", 4096, NULL, 4, &wifi_task_handle);
    BaseType_t status_sensor = xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 3, &sensor_task_handle);
    BaseType_t status_mqtt = xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 3, &mqtt_task_handle);

    if (status_wifi != pdPASS || status_sensor != pdPASS || status_mqtt != pdPASS) {
        printf("ERRO CRITICO: Nao ha memoria suficiente para criar as tarefas!\n");
        for (;;) {
            set_led_rgb(true, false, false);
            sleep_ms(100);
            set_led_rgb(false, false, false);
            sleep_ms(100);
        }
    }

    printf("Tarefas criadas. Iniciando Scheduler...\n");
    vTaskStartScheduler();

    for (;;) {
        tight_loop_contents();
    }
}
