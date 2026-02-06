# FreeRTOS Kernel import helper for Pico SDK

if (NOT FREERTOS_KERNEL_PATH)
    if (DEFINED ENV{FREERTOS_KERNEL_PATH})
        set(FREERTOS_KERNEL_PATH $ENV{FREERTOS_KERNEL_PATH})
    elseif (DEFINED PICO_SDK_PATH)
        set(FREERTOS_KERNEL_PATH ${PICO_SDK_PATH}/lib/FreeRTOS-Kernel)
        if (NOT EXISTS ${FREERTOS_KERNEL_PATH})
            set(FREERTOS_KERNEL_PATH ${PICO_SDK_PATH}/external/FreeRTOS-Kernel)
        endif()
    endif()
endif()

if (NOT FREERTOS_KERNEL_PATH OR NOT EXISTS ${FREERTOS_KERNEL_PATH})
    message(FATAL_ERROR "FreeRTOS Kernel path not found. Set FREERTOS_KERNEL_PATH.")
endif()

include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
