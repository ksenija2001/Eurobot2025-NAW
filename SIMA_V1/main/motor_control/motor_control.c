#include "motor_control.h"

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
    ESP_LOGI("Servo", "Duty Cycle: %f, Angle: %f", dc, angle);
    mcpwm_set_signal_low(servo.unit, servo.timer, MCPWM_GEN_A);
    mcpwm_set_duty(servo.unit, servo.timer, MCPWM_GEN_A, dc);
    mcpwm_set_duty_type(servo.unit, servo.timer, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
}