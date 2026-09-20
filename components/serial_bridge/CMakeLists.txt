idf_component_register(SRCS "serial_bridge.c"
                       INCLUDE_DIRS "include"
                       REQUIRES protocol motor_driver driver esp_driver_gpio)