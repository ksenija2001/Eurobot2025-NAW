#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#include "esp_log.h"
#include "driver/i2c_master.h"

typedef struct {
    i2c_master_bus_config_t bus_config;
    i2c_master_bus_handle_t bus_handle;
} I2C_Bus;

typedef struct {
    i2c_device_config_t     dev_config;
    i2c_master_dev_handle_t dev_handle;
} I2C_Device;

void init_i2c_bus(I2C_Bus* bus, uint8_t SDA_PIN, uint8_t SCL_PIN, uint8_t glitch_ignore_cnt);

void init_i2c_device(I2C_Bus* bus, I2C_Device* dev, uint8_t dev_address, uint32_t speed);

esp_err_t i2c_device_alive(I2C_Bus* bus, uint16_t address);

void i2c_send(I2C_Device* dev, uint16_t address, uint8_t* data, uint8_t len);
void i2c_sendByte(I2C_Device* dev, uint16_t address, uint8_t data);
void i2c_sendWord(I2C_Device* dev, uint16_t address, uint16_t data);

void i2c_receive(I2C_Device* dev, uint16_t address, uint8_t* buff, uint8_t len);
void i2c_receiveByte(I2C_Device* dev, uint16_t address, uint8_t* buff);
void i2c_receiveWord(I2C_Device* dev, uint16_t address, uint8_t* buff);

#endif //I2C_H