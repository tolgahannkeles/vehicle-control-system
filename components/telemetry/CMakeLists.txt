idf_component_register(SRCS "telemetry.c"
                       INCLUDE_DIRS "include"
                       REQUIRES protocol serial_bridge imu_sensor gps_sensor)