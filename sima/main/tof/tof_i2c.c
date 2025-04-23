/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "tof_i2c.h"

#include "esp_log.h"

void init_tof_i2c(I2C_Bus* bus, VL53LMZ_Platform* platform){
	#ifdef DEBUG_TOF_I2C
		ESP_LOGI("TOF_I2C", "Initializin I2C for TOF with address %x", platform->address);
	#endif
	init_i2c_device(bus, &platform->device, platform->address, platform->speed);
}

uint8_t RdByte(
	VL53LMZ_Platform *p_platform,
	uint16_t RegisterAdress,
	uint8_t *p_value)
{
  return p_platform->Read(&p_platform->device, RegisterAdress, p_value, 1U);
}

uint8_t WrByte(
	VL53LMZ_Platform *p_platform,
	uint16_t RegisterAdress,
	uint8_t value)
{
  return p_platform->Write(&p_platform->device, RegisterAdress, &value, 1U);
}

uint8_t WrMulti(
	VL53LMZ_Platform *p_platform,
	uint16_t RegisterAdress,
	uint8_t *p_values,
	uint32_t size)
{
  return p_platform->Write(&p_platform->device, RegisterAdress, p_values, size);
}

uint8_t RdMulti(
	VL53LMZ_Platform *p_platform,
	uint16_t RegisterAdress,
	uint8_t *p_values,
	uint32_t size)
{
  return p_platform->Read(&p_platform->device, RegisterAdress, p_values, size);
}

void SwapBuffer(
    uint8_t     *buffer,
    uint16_t     size)
{
  uint32_t i, tmp;

  /* Example of possible implementation using <string.h> */
  for(i = 0; i < size; i = i + 4)
  {
    tmp = (
      buffer[i]<<24)
    |(buffer[i+1]<<16)
    |(buffer[i+2]<<8)
    |(buffer[i+3]);

    memcpy(&(buffer[i]), &tmp, 4);
  }
}

uint8_t WaitMs(
	VL53LMZ_Platform *p_platform,
	uint32_t TimeMs)
{
  // uint32_t tickstart;
  // tickstart = p_platform->GetTick();

  // while ((p_platform->GetTick() - tickstart) < TimeMs);
  vTaskDelay(pdMS_TO_TICKS(TimeMs));
  return 0;
}

