idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash motor_driver serial_bridge telemetry wifi_ap web_ui protocol imu_sensor)