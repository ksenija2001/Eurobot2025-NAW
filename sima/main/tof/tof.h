#ifndef TOF_H
#define TOF_H

#define DEBUG_TOF

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_system.h"

#include "../gpio/gpio.h"
#include "../i2c/i2c.h"

#include "tof_buffer.h"

#define TOF_TAG_MAX 32

typedef struct{
    char tag[TOF_TAG_MAX];

    I2C_Bus* bus;
    I2C_Device device;

    uint8_t address;
    uint32_t speed;

    uint8_t i2c_rst_pin;

    uint8_t temp_buffer[(uint16_t)(-1)];
    uint8_t offset_data[(uint16_t)(-1)];

    uint8_t* default_xtalk;
    uint8_t xtalk_data[776];

    uint8_t* default_configuration;
} TOF;

void init_tof(TOF* tof, I2C_Bus* bus, uint8_t address, uint8_t rst);
void tof_is_alive(TOF* tof);

void tof_sendByte(TOF* tof, uint16_t address, uint8_t data);
void tof_send(TOF* tof, uint16_t address, uint8_t* data, uint16_t size);

void tof_receive(TOF* tof, uint16_t address, uint8_t* data, uint16_t size);

void tof_firmware(TOF* tof);

#endif //TOF_H