#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdio.h>
#include <stdint.h>
#include "mcpwm/mcpwm.h"
#include "esp_log.h"

typedef enum {
    SERVO_CHECK,
    SERVO_GO_TO_TOP,
    SERVO_GO_TO_BOTTOM
} servo_states_e;

void motor_forward(motor_t motor, float dc);
void motor_back(motor_t motor, float dc);
void motors_stop(motor_t motor1, motor_t motor2);
void servo_move(motor_t servo, float angle);
void servo_wave(motor_t servo1, motor_t servo2, float ang11, float ang12, float ang21, float ang22);

#endif  //MOTOR_CONTROL_H