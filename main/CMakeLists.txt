idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash motor_driver wifi_ap web_ui uart_receiver ota_manager
)