/*
 * imu_regs.h
 *
 *  Created on: Dec 20, 2024
 *  Modified on: Jan 08, 2025
 *       Author: Filip Goldberger
 */

#ifndef INC_IMU_REGS_H_
#define INC_IMU_REGS_H_

/** --------------------------------------------- **/
/** ---         IMU_WHO_AM_I_REG              --- **/
/** --------------------------------------------- **/
#define IMU_WHO_AM_I_REG 0x0f


/** --------------------------------------------- **/
/** ---         CONTROL REGISTERS             --- **/
/** --------------------------------------------- **/
#define IMU_CTRL1_XL_REG 0x10	// Accelerometer register
#define IMU_CTRL2_G_REG  0x11	// Gyroscope register

#define IMU_CTRL3_C_REG  0x12	// IMU RESET
#define IMU_CTRL4_C_REG  0x13	// IMU control register

#define IMU_CTRL9_XL_REG 0x18	// IMU PROPER device configuration

/** --------------------------------------------- **/
/** ---            IMU Modes                  --- **/
/** --------------------------------------------- **/
#define IMU_SLEEP_G_ENABLE 0x40
#define IMU_SLEEP_G_DISABLE 0x00

/** --------------------------------------------- **/
/** ---    IMU accelerometet and gyroscope    --- **/
/** ---    data registers start (6 bytes)     --- **/
/** --------------------------------------------- **/
#define IMU_DATA_GYR_ACC 0x22


/** --------------------------------------------- **/
/** ---    Gyroscope registers start          --- **/
/** --------------------------------------------- **/
#define IMU_GYR_OUT 0x22

/** --------------------------------------------- **/
/** ---        Gyroscope X value              --- **/
/** --------------------------------------------- **/
#define IMU_GYR_OUT_X_L 0x22
#define IMU_GYR_OUT_X_H 0x23
#define IMU_GYR_OUT_X IMU_GYR_OUT_X_L


/** --------------------------------------------- **/
/** ---        Gyroscope Y value              --- **/
/** --------------------------------------------- **/
#define IMU_GYR_OUT_Y_L 0x24
#define IMU_GYR_OUT_Y_H 0x25
#define IMU_GYR_OUT_Y IMU_GYR_OUT_Y_L


/** --------------------------------------------- **/
/** ---        Gyroscope Z value              --- **/
/** --------------------------------------------- **/
#define IMU_GYR_OUT_Z_L 0x26
#define IMU_GYR_OUT_Z_H 0x27
#define IMU_GYR_OUT_Z IMU_GYR_OUT_Z_L


/** --------------------------------------------- **/
/** ---    Accelerometer registers start      --- **/
/** --------------------------------------------- **/
#define IMU_ACC_OUT 0x28

/** --------------------------------------------- **/
/** ---      Accelerometer X value            --- **/
/** --------------------------------------------- **/
#define IMU_ACC_OUT_X_L 0x28
#define IMU_ACC_OUT_X_H 0x29
#define IMU_ACC_OUT_X IMU_ACC_OUT_X_L


/** --------------------------------------------- **/
/** ---      Accelerometer Y value            --- **/
/** --------------------------------------------- **/
#define IMU_ACC_OUT_Y_L 0x2A
#define IMU_ACC_OUT_Y_H 0x2B
#define IMU_ACC_OUT_Y IMU_ACC_OUT_Y_L


/** --------------------------------------------- **/
/** ---      Accelerometer Z value            --- **/
/** --------------------------------------------- **/
#define IMU_ACC_OUT_Z_L 0x2C
#define IMU_ACC_OUT_Z_H 0x2D
#define IMU_ACC_OUT_Z IMU_ACC_OUT_Z_L


/** --------------------------------------------- **/
/** ---    Configuration for accelerometer    --- **/
/** ---           output data rate            --- **/
/** --------------------------------------------- **/
#if defined(IMU_ACC_POWER_DOWN)
	#define IMU_ACC_ODR_CONFIG 0x0
#elif  defined(IMU_ACC_ODR_1_6_HZ)
	#define IMU_ACC_ODR_CONFIG 0xb
#elif  defined(IMU_ACC_ODR_12_5_HZ)
	#define IMU_ACC_ODR_CONFIG 0x1
#elif  defined(IMU_ACC_ODR_26_HZ)
	#define IMU_ACC_ODR_CONFIG 0x2
#elif  defined(IMU_ACC_ODR_52_HZ)
	#define IMU_ACC_ODR_CONFIG 0x3
#elif  defined(IMU_ACC_ODR_104_HZ)
	#define IMU_ACC_ODR_CONFIG 0x4
#elif  defined(IMU_ACC_ODR_208_HZ)
	#define IMU_ACC_ODR_CONFIG 0x5
#elif  defined(IMU_ACC_ODR_416_HZ)
	#define IMU_ACC_ODR_CONFIG 0x6
#elif  defined(IMU_ACC_ODR_833_HZ)
	#define IMU_ACC_ODR_CONFIG 0x7
#elif  defined(IMU_ACC_ODR_1_66_KHZ)
	#define IMU_ACC_ODR_CONFIG 0x8
#elif  defined(IMU_ACC_ODR_3_33_KHZ)
	#define IMU_ACC_ODR_CONFIG 0x9
#elif  defined(IMU_ACC_ODR_6_66_KHZ)
	#define IMU_ACC_ODR_CONFIG 0xA
#endif


/** --------------------------------------------- **/
/** ---    Configuration for accelerometer    --- **/
/** ---             sensitivity               --- **/
/** --------------------------------------------- **/
#ifdef IMU_ACC_FS_2G
	#define IMU_ACC_FS_CONFIG 0b00
	#define IMU_ACC_CONVERSION 0x04
#elif defined(IMU_ACC_FS_4G)
	#define IMU_ACC_FS_CONFIG 0b10
	#define IMU_ACC_CONVERSION 0x08
#elif  defined(IMU_ACC_FS_8G)
	#define IMU_ACC_FS_CONFIG 0b11
	#define IMU_ACC_CONVERSION 0x10
#elif  defined(IMU_ACC_FS_16G)
	#define IMU_ACC_FS_CONFIG 0b01
	#define IMU_ACC_CONVERSION 0x20
#endif


/** --------------------------------------------- **/
/** ---      Configuration for gyroscope      --- **/
/** ---           output data rate            --- **/
/** --------------------------------------------- **/
#if defined(IMU_GYR_POWER_DOWN)
	#define IMU_GYR_ODR_CONFIG 0x0
#elif  defined(IMU_GYR_ODR_12_5_HZ)
	#define IMU_GYR_ODR_CONFIG 0x1
#elif  defined(IMU_GYR_ODR_26_HZ)
	#define IMU_GYR_ODR_CONFIG 0x2
#elif  defined(IMU_GYR_ODR_52_HZ)
	#define IMU_GYR_ODR_CONFIG 0x3
#elif  defined(IMU_GYR_ODR_104_HZ)
	#define IMU_GYR_ODR_CONFIG 0x4
#elif  defined(IMU_GYR_ODR_208_HZ)
	#define IMU_GYR_ODR_CONFIG 0x5
#elif  defined(IMU_GYR_ODR_416_HZ)
	#define IMU_GYR_ODR_CONFIG 0x6
#elif  defined(IMU_GYR_ODR_833_HZ)
	#define IMU_GYR_ODR_CONFIG 0x7
#elif  defined(IMU_GYR_ODR_1_66_KHZ)
	#define IMU_GYR_ODR_CONFIG 0x8
#elif  defined(IMU_GYR_ODR_3_33_KHZ)
	#define IMU_GYR_ODR_CONFIG 0x9
#elif  defined(IMU_GYR_ODR_6_66_KHZ)
	#define IMU_GYR_ODR_CONFIG 0xA
#endif


/** --------------------------------------------- **/
/** ---      Configuration for gyroscope      --- **/
/** ---             sensitivity               --- **/
/** --------------------------------------------- **/
#ifdef IMU_GYR_FS_250
	#define IMU_GYR_FS_CONFIG 0b00
#elif  defined(IMU_GYR_FS_500)
	#define IMU_GYR_FS_CONFIG 0b10
#elif  defined(IMU_GYR_FS_1000)
	#define IMU_GYR_FS_CONFIG 0b11
#elif  defined(IMU_GYR_FS_2000)
	#define IMU_GYR_FS_CONFIG 0b01
#endif

#endif /* INC_IMU_REGS_H_ */
