/*
 * ism330dhcx_regs.h
 *
 *  Created on: Feb 4, 2025
 *      Author: filip
 */

#ifndef INC_ISM330DHCX_REGS_H_
#define INC_ISM330DHCX_REGS_H_

#define ISM_REG_WHOAMI 0x0f
#define ISM_REG_DATA_WHOAMI 0x6b
#define ISM_REG_STATUS 0x1e

#define ISM_REG_CTRL1_XL 0x10
#define ISM_REG_CTRL2_G  0x11
#define ISM_REG_CTRL3_C  0x12
#define ISM_REG_CTRL4_C  0x13
#define ISM_REG_CTRL5_C  0x14
#define ISM_REG_CTRL6_C  0x15
#define ISM_REG_CTRL7_G  0x16
#define ISM_REG_CTRL8_XL 0x17
#define ISM_REG_CTRL9_XL 0x18
#define ISM_REG_CTRL10_C 0x19

#define ISM_REG_OUT_TEMP_L 0x20
#define ISM_REG_OUT_TEMP_H 0x21

#define ISM_REG_OUT		 0x22
#define ISM_REG_OUT_GYRO 0x22
#define ISM_REG_OUT_ACC	 0x28

#define ISM_REG_OUTX_L_G 0x22
#define ISM_REG_OUTX_H_G 0x23

#define ISM_REG_OUTY_L_G 0x24
#define ISM_REG_OUTY_H_G 0x25

#define ISM_REG_OUTZ_L_G 0x26
#define ISM_REG_OUTZ_H_G 0x27

#define ISM_REG_OUTX_L_A 0x28
#define ISM_REG_OUTX_H_A 0x29

#define ISM_REG_OUTY_L_A 0x2A
#define ISM_REG_OUTY_H_A 0x2B

#define ISM_REG_OUTZ_L_A 0x2C
#define ISM_REG_OUTZ_H_A 0x2D

#endif /* INC_ISM330DHCX_REGS_H_ */
