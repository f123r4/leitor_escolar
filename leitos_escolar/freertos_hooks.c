#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

#define LED_R_PIN 13

static void panic_blink(uint32_t delay_cycles) {
    gpio_init(LED_R_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);

    taskDISABLE_INTERRUPTS();

    for (;;) {
        gpio_put(LED_R_PIN, 1);
        for (volatile uint32_t i = 0; i < delay_cycles; i++) {
            tight_loop_contents();
        }
        gpio_put(LED_R_PIN, 0);
        for (volatile uint32_t i = 0; i < delay_cycles; i++) {
            tight_loop_contents();
        }
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name) {
    (void) task;

    printf("\n!!! PANICO !!!\n");
    printf("Estouro de pilha na tarefa: %s\n", task_name ? task_name : "(desconhecida)");
    stdio_flush();

    panic_blink(1000000u);
}

void vApplicationMallocFailedHook(void) {
    printf("\n!!! PANICO !!!\n");
    printf("Falha de alocacao de memoria (Malloc Failed)\n");
    stdio_flush();

    panic_blink(5000000u);
}
