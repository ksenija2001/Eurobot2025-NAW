#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <string.h>

#include "driver/i2c.h"

//#define DEBUG_I2C
#define LOW_DEBUG_I2C_LEVEL 0
#define HIGH_DEBUG_I2C_LEVEL 2

#if defined(DEBUG_I2C)
    #define I2C_TAG "I2C"
    
    #include "esp_log.h"
#endif

#if !defined(DEBUG_I2C_LEVEL)
    #define DEBUG_I2C_LEVEL HIGH_DEBUG_I2C_LEVEL
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

/***
 * @brief Functions used to initialize I2C driver on i2c_port 0
 * 
 * @param SCL_PIN GPIO pin used for I2C clock line
 * @param SDA_PIN GPIO pin used for I2C data line 
 * 
 * @retval None
 */
void init_i2c0(gpio_num_t SCL_PIN, gpio_num_t SDA_PIN);

/***
 * @brief FUNCTION NOT IN FUNCTION !!!
 * 
 * Function used to send data from I2C driver on i2c_port 0
 * 
 * @param data      Pointer to data to be sent
 * @param data_len  Length of data
 * @param ack_en    Set to 1 if ACK should be sent from slave, otherwise 0
 * 
 * @retval Return ESP_OK (0) if everything is ok
 */
esp_err_t i2c0_send(uint8_t* data, uint32_t data_len, uint8_t ack_en);

/***
 * @brief FUNCTION NOT IN FUNCTION !!!
 * 
 * Function used to receive data from I2C driver on i2c_port 0
 * 
 * @param data      Pointer to buffer to be read
 * @param data_len  Length of buffer
 * @param ack_en    Set to 1 if ACK should be sent from slave, otherwise 0
 * 
 * @retval Return ESP_OK (0) if everything is ok
 */
esp_err_t i2c0_receive(uint8_t* buff, uint32_t buff_len, uint8_t ack_en);

/***
 * @brief Function used to set register value on slave device where address of register is 16 bit
 * 
 * @param dev_addr  Slave device address
 * @param reg_addr  16 bit register address
 * @param data      Pointer to data to be sent
 * @param data_len  Length of data
 * 
 * @retval Return ESP_OK (0) if everything is ok
 */
esp_err_t i2c0_send_to_reg16(uint16_t dev_addr, uint16_t reg_addr, uint8_t* data, uint32_t data_len);

/***
 * @brief Function used to get register value on slave device where address of register is 16 bit
 * 
 * @param dev_addr  Slave device address
 * @param reg_addr  16 bit register address
 * @param buff      Pointer to buffer where data will be stored
 * @param buff_len  Length of buffer
 * 
 * @retval Return ESP_OK (0) if everything is ok
 */
esp_err_t i2c0_receive_from_reg16(uint16_t dev_addr, uint16_t reg_addr, uint8_t* buff, uint32_t buff_len);

#endif //I2C_H