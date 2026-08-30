idf_component_register(
    SRCS "uart_receiver.c"
    INCLUDE_DIRS "include"
    REQUIRES motor_driver esp_driver_uart log
)