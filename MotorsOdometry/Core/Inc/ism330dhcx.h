/*
 * ism330dhcx.h
 *
 *  Created on: Feb 4, 2025
 *      Author: filip
 */

#ifndef INC_ISM330DHCX_H_
#define INC_ISM330DHCX_H_

#include "ism330dhcx_regs.h"
#include "ism330dhcx_utils.h"
#include "ism330dhcx_settings.h"

// (0b1101011) << 1
// (WHO_AM_I) << 1
#define ISM_ADDRESS 0xd6

ISM330DHCX_Status init_ISM330DHCX(ISM330DHCX *ism, uint8_t address, I2C_HandleTypeDef* i2c);
extern ISM330DHCX_Status __reset_I2C1_LINE();

ISM330DHCX_Status set_Fullscale_Gyroscope(ISM330DHCX *ism, ISM330DHCX_FS_GYROSCOPE fs);
ISM330DHCX_Status set_Fullscale_Accelerometer(ISM330DHCX *ism, ISM330DHCX_FS_ACCELEROMETER fs);

ISM330DHCX_Status set_OutputDataRate_Gyroscope(ISM330DHCX *ism, ISM330DHCX_ODR odr);
ISM330DHCX_Status set_OutputDataRate_Accelerometer(ISM330DHCX *ism, ISM330DHCX_ODR odr);

ISM330DHCX_Status get_Axies_Raw_All(ISM330DHCX *ism);
ISM330DHCX_Status get_Axies_Raw_Gyroscope(ISM330DHCX *ism);
ISM330DHCX_Status get_Axies_Raw_Accelerometer(ISM330DHCX *ism);

ISM330DHCX_Status get_Axies_All(ISM330DHCX *ism, float ms);
ISM330DHCX_Status get_Axies_Gyroscope(ISM330DHCX *ism, float ms);
ISM330DHCX_Status get_Axies_Accelerometer(ISM330DHCX *ism, float ms);

ISM330DHCX_Status convert_Raw(ISM330DHCX* ism, float ms);

Angle convert_Raw_Gyroscope(ISM330DHCX* ism, float ms);
Acceleration convert_Raw_Accelerometer(ISM330DHCX* ism, float ms);

extern Angle (*user_convert_Raw_Gyroscope)(ISM330DHCX* ism, float ms);
extern Acceleration (*user_convert_Raw_Accelerometer)(ISM330DHCX* ism, float ms);

#endif /* INC_ISM330DHCX_H_ */
