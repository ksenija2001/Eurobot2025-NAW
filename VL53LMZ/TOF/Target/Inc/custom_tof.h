/*
 * custom_tof.h
 *
 *  Created on: Feb 6, 2025
 *      Author: xenia
 */

#ifndef TARGET_INC_CUSTOM_TOF_H_
#define TARGET_INC_CUSTOM_TOF_H_

#include "custom_bus.h"
#include "vl53lmz_api.h"
#include "vector.h"
#include "main.h"

typedef struct {
	GPIO_TypeDef* LPn_port;
	uint16_t LPn_pin;

	GPIO_TypeDef* RST_port;
	uint16_t RST_pin;

	GPIO_TypeDef* PWR_EN_port;
	uint16_t PWR_EN_pin;
} VL53LMZ_IO;

typedef struct
{
  uint32_t Distance; /*!< millimeters */
  uint32_t Status;   /*!< OK: 0, NOK: !0 */
  float Ambient;   /*!< kcps / spad */
  float Signal;    /*!< kcps / spad */
} VL53LMZ_ZoneResult_t;

typedef struct
{
  uint32_t NumberOfZones;
  VL53LMZ_ZoneResult_t ZoneResult[VL53LMZ_RESOLUTION_8X8];
} VL53LMZ_Result_t;

typedef struct {
	VL53LMZ_Configuration conf;
	VL53LMZ_IO io;
	sVector3_t trans_offset;
	sVector3_t orient_offset;
	sVector3_t point_cloud[64];
} VL53LMZ_Object;

uint8_t VL53LMZ_Init(VL53LMZ_Object* dev, uint16_t address);
void VL53LMZ_Reset(VL53LMZ_IO* io);
uint8_t VL53LMZ_Config(VL53LMZ_Configuration* conf, uint8_t resolution, uint8_t ranging_mode, uint32_t integration_time, uint8_t ranging_frequency, uint8_t sharpener);
uint8_t VL53LMZ_Start_Ranging(VL53LMZ_Configuration* conf);
uint8_t VL53LMZ_Get_Distance(VL53LMZ_Configuration* conf, VL53LMZ_Result_t* result);

uint8_t VL53LMZ_Get_Distance_IT(VL53LMZ_Configuration* conf, VL53LMZ_ResultsData* data);
void VL53LMZ_Get_Result(VL53LMZ_ResultsData raw, uint8_t resolution, VL53LMZ_Result_t* data);




#endif /* TARGET_INC_CUSTOM_TOF_H_ */
