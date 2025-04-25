#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <string.h>

#include "driver/i2c.h"

#define DEBUG_I2C
#define DEBUG_I2C_LEVEL 1

#if defined(DEBUG_I2C)
    #define I2C_TAG "I2C"
    
    #include "esp_log.h"
#endif

#if !defined(DEBUG_I2C_LEVEL)
    #define DEBUG_I2C_LEVEL 1
#endif

#define ACK_EN 1
#define ACK_DIS 0

#define START_BIT_EN 0b01
#define START_BIT_DIS 0b00

#define STOP_BIT_EN 0b10
#define STOP_BIT_DIS 0b00

#define START_OK_STRING "Command start ok"
#define WRITE_OK_STRING "Command write ok"
#define READ_OK_STRING "Command read ok"
#define STOP_OK_STRING "Command stop ok"
#define COMMAND_BEGIN_OK_STRING "Command begin ok"

#define ESP_ERR_STRING "ESP_ERR"
#define ESP_FAIL_STRING "ESP_FAIL | ACK NOT RECEIVED"
#define ESP_ERR_INVALID_ARG_STRING "ESP_ERR_INVALID_ARG"

void init_i2c0(gpio_num_t SCL_PIN, gpio_num_t SDA_PIN);

uint32_t i2c0_send(uint16_t dev_addr, uint16_t reg_addr, uint8_t* data, uint32_t data_len);
uint32_t i2c0_receive(uint16_t dev_addr, uint16_t reg_addr, uint8_t* buff, uint32_t buff_len);

#endif //I2C_H