#ifndef MAIN_H
#define MAIN_H

#include "freertos/FreeRTOS.h"

#include "credentials.h"

#include "wifi/wifi.h"
#include "socket/socket.h"

#include "i2c/i2c.h"

#include "tof/tof.h"
#include "tof/point_cloud.h"

#include "utils/utils.h"

#include "servo/servo.h"

#define I2C_TOF_ADDRESS 0x52
#define TOF_NUMBER_OF_ZONES VL53LMZ_RESOLUTION_4X4

/***
 * @brief Function running on second core
 */
void app_main_2();

#endif //MAIN_H