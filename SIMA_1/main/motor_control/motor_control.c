#include "motor_control.h"

float current_angle_1 = 30.0;
float current_angle_2 = 100.0;

servo_states_e servo_state = SERVO_CHECK;

void motor_forward(motor_t motor, float dc)
{
    mcpwm_set_signal_low(motor.unit, motor.timer, MCPWM_GEN_B);
    //mcpwm_set_duty(motor.unit, motor.timer, MCPWM_GEN_B, 0.0);
    mcpwm_set_duty(motor.unit, motor.timer, MCPWM_GEN_A, dc);
    mcpwm_set_duty_type(motor.unit, motor.timer, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
}

void motor_back(motor_t motor, float dc)
{
    mcpwm_set_signal_low(motor.unit, motor.timer, MCPWM_GEN_A);
    //mcpwm_set_duty(motor.unit, motor.timer, MCPWM_GEN_A, 0.0);
    mcpwm_set_duty(motor.unit, motor.timer, MCPWM_GEN_B, dc);
    mcpwm_set_duty_type(motor.unit, motor.timer, MCPWM_GEN_B, MCPWM_DUTY_MODE_0);
}

void motors_stop(motor_t motor1, motor_t motor2)
{
    /*motor_forward(motor1, 0.0);
    motor_forward(motor2, 0.0);*/
    mcpwm_set_signal_high(motor2.unit, motor2.timer, MCPWM_GEN_A);
    mcpwm_set_signal_high(motor2.unit, motor2.timer, MCPWM_GEN_B);
    mcpwm_set_signal_high(motor1.unit, motor1.timer, MCPWM_GEN_A);
    mcpwm_set_signal_high(motor1.unit, motor1.timer, MCPWM_GEN_B);
    // mcpwm_set_signal_low(motor1.unit, motor1.timer, MCPWM_GEN_A);
    // mcpwm_set_signal_low(motor1.unit, motor1.timer, MCPWM_GEN_B);
    // mcpwm_set_signal_low(motor2.unit, motor2.timer, MCPWM_GEN_A);
    // mcpwm_set_signal_low(motor2.unit, motor2.timer, MCPWM_GEN_B);
}

void servo_move(motor_t servo, float angle)
{
    float dc = (angle*10.0)/180.0 + 2.5;
    if (dc > 12.5) dc = 12.5;
    else if (dc < 2.5) dc = 2.5;
    //ESP_LOGI("Servo", "Duty Cycle: %f, Angle: %f", dc, angle);
    mcpwm_set_signal_low(servo.unit, servo.timer, MCPWM_GEN_A);
    mcpwm_set_duty(servo.unit, servo.timer, MCPWM_GEN_A, dc);
    mcpwm_set_duty_type(servo.unit, servo.timer, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
}

void servo_wave(motor_t servo1, motor_t servo2, float ang11, float ang12, float ang21, float ang22)
{
    static bool top_angle= false;
    static bool bottom_angle = false;
    static bool start_servo = true;
    switch (servo_state)
    {
        case SERVO_CHECK:
            if (start_servo)
            {
                servo_state = SERVO_GO_TO_TOP;
                start_servo = false;
            }
            //Ovo ispod se moze resiti sa jednim flegom
            if (!top_angle) servo_state = SERVO_GO_TO_TOP;
            if (!bottom_angle) servo_state = SERVO_GO_TO_BOTTOM;
        break;
        case SERVO_GO_TO_TOP:
            if (current_angle_2 < ang22)
            {
                servo_move(servo2, current_angle_2);
                current_angle_2++;
            }
            if (current_angle_1 < ang11)
            {
                servo_move(servo1, current_angle_1);
                current_angle_1++;
            }
            if (current_angle_1 == ang11 && current_angle_2 == ang22)
            {
                top_angle = true;
                bottom_angle = false;
                servo_state = SERVO_CHECK;
            }
        break;
        case SERVO_GO_TO_BOTTOM:
            if (current_angle_2 > ang21)
            {
                servo_move(servo2, current_angle_2);
                current_angle_2--;
            }
            if (current_angle_1 > ang12)
            {
                servo_move(servo1, current_angle_1);
                current_angle_1--;
            }
            if (current_angle_1 == ang12 && current_angle_2 == ang21)
            {
                top_angle = false;
                bottom_angle = true;
                servo_state = SERVO_CHECK;
            }
        break;
    }
}