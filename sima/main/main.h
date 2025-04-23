#ifndef MAIN_H
#define MAIN_H

#include "freertos/FreeRTOS.h"

#include "credentials.h"

#include "wifi/wifi.h"
#include "socket/socket.h"

#include "i2c/i2c.h"

#include "tof/tof.h"
#include "tof/tof_i2c.h"
#include "tof/tof_api.h"
#include "tof/point_cloud.h"

#define I2C_TOF_ADDRESS (0x52 >> 1)

#endif //MAIN_H