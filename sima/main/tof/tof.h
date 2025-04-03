#ifndef TOF_H
#define TOF_H

#define DEBUG_TOF

#include "freertos/FreeRTOS.h"

#include "../gpio/gpio.h"
#include "../i2c/i2c.h"

#define TOF_TAG_MAX 32

typedef struct{
    char tag[TOF_TAG_MAX];

    I2C_Bus* bus;
    I2C_Device device;

    uint8_t address;
    uint32_t speed;

    uint8_t i2c_rst_pin;
} TOF;

void init_tof(TOF* tof, I2C_Bus* bus, uint8_t address, uint8_t rst);
void tof_is_alive(TOF* tof);

void tof_configure(TOF* tof);

void tof_send(TOF* tof);
void tof_receive(TOF* tof);

#endif //TOF_H