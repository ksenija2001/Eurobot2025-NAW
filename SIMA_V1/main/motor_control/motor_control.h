#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdio.h>
#include <stdint.h>
#include "mcpwm/mcpwm.h"
#include "esp_log.h"

void motor_forward(motor_t motor, float dc);
void motor_back(motor_t motor, float dc);
void motors_stop(motor_t motor1, motor_t motor2);
void servo_move(motor_t servo, float angle);

#endif  //MOTOR_CONTROL_H