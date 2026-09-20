#pragma once

#include <stdbool.h>

void motor_init(void);

// RPi / ROS 2 için (m/s cinsinden bağımsız palet hızları)
void motor_set_targets(float left_mps, float right_mps);

// Web UI / Joystick için (-1000..1000 aralığında throttle ve steer)
void motor_update(int throttle, int steer);

void motor_stop(void);