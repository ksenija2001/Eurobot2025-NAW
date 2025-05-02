/*
 * custom_tof.h
 *
 *  Created on: Feb 6, 2025
 *      Author: xenia
 */

#ifndef TARGET_INC_CUSTOM_TOF_H_
#define TARGET_INC_CUSTOM_TOF_H_

#include "tof_api.h"
#include "../utils/vector.h"

#include "tof_i2c.h"
#include "../gpio/gpio.h"

#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#include "esp_log.h"

#define DEBUG_TOF

#define TOF_NUMBER_OF_ZONES VL53LMZ_RESOLUTION_4X4

#if defined(TOF_NUMBER_OF_ZONES) && TOF_NUMBER_OF_ZONES == VL53LMZ_RESOLUTION_8X8
	#define TOF_RIGHT_ZONE_1 24
	#define TOF_RIGHT_ZONE_2 32

	#define TOF_LEFT_ZONE_1 31
	#define TOF_LEFT_ZONE_2 39

	#define TOF_CENTER_ZONE_1 35
	#define TOF_CENTER_ZONE_2 27
#else
	#define TOF_RIGHT_ZONE_1 4
	#define TOF_RIGHT_ZONE_2 8

	#define TOF_LEFT_ZONE_1 7
	#define TOF_LEFT_ZONE_2 11

	#define TOF_CENTER_ZONE_1 5
	#define TOF_CENTER_ZONE_2 9
#endif

#define TOF_INTERRUPT_DISTANCE 300

typedef struct {
	//GPIO_TypeDef* LPn_port;
	//uint16_t LPn_pin;
	gpio_num_t LPn_pin;

	// GPIO_TypeDef* RST_port;
	// uint16_t RST_pin;
	gpio_num_t RST_pin;


	// GPIO_TypeDef* PWR_EN_port;
	// uint16_t PWR_EN_pin;
	gpio_num_t PWR_EN_pin;

	gpio_num_t INTR_pin;
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

	bool interrupt;

	bool interrupt_left_zone;
	bool interrupt_right_zone;
	bool interrupt_center_zone;
} VL53LMZ_Object;

typedef struct {
	bool interrupt_left;
	bool interrupt_right;
	bool interrupt_center;
} VL53LMZ_Interrupt_Zone;

uint8_t VL53LMZ_Init(VL53LMZ_Object* dev, uint16_t address);
void VL53LMZ_Reset(VL53LMZ_IO* io);
uint8_t VL53LMZ_Config(VL53LMZ_Configuration* conf, uint8_t resolution, uint8_t ranging_mode, uint32_t integration_time, uint8_t ranging_frequency, uint8_t sharpener);
uint8_t VL53LMZ_Start_Ranging(VL53LMZ_Configuration* conf);
uint8_t VL53LMZ_Get_Distance(VL53LMZ_Configuration* conf, VL53LMZ_Result_t* result);

uint8_t VL53LMZ_Get_Distance_IT(VL53LMZ_Configuration* conf, VL53LMZ_ResultsData* data);
void VL53LMZ_Get_Result(VL53LMZ_ResultsData raw, uint8_t resolution, VL53LMZ_Result_t* data);

/***
 * @brief This function initializes interrupt for ToF new data interrupt
 * 
 * @param num GPIO pin number on which interrupt should happen
 * @param tof Pointer to ToF object
 * 
 * @retval None
 */
void init_tof_intr(gpio_num_t num, VL53LMZ_Object* tof);

/***
 * @brief This function checks if interrupt has happened
 * 
 * @param tof Pointer to ToF object
 * 
 * @retval Return true if interrupt has happened, otherwise false
 */
bool get_tof_intr(VL53LMZ_Object* tof);

void tof_calculate_distances_interrupt(VL53LMZ_Object* tof, VL53LMZ_Result_t *data);
bool get_tof_intr_zone(VL53LMZ_Object* tof, VL53LMZ_Result_t *data, VL53LMZ_Interrupt_Zone* zone);

#endif /* TARGET_INC_CUSTOM_TOF_H_ */
