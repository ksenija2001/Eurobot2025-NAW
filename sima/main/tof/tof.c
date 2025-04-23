/*
 * custom_tof.c
 *
 *  Created on: Feb 6, 2025
 *      Author: xenia
 */

#include "tof.h"

void VL53LMZ_Reset(VL53LMZ_IO* io){
	/* Enable power */
	init_gpio(GPIO_MODE_OUTPUT, GPIO_INTR_DISABLE, io->PWR_EN_pin, GPIO_PULLDOWN_DISABLE, GPIO_PULLUP_DISABLE);
	init_gpio(GPIO_MODE_OUTPUT, GPIO_INTR_DISABLE, GPIO_NUM_15, GPIO_PULLDOWN_DISABLE, GPIO_PULLUP_DISABLE);
	gpio_reset(io->PWR_EN_pin);

	vTaskDelay(pdMS_TO_TICKS(100));

	gpio_set(io->PWR_EN_pin);

	vTaskDelay(pdMS_TO_TICKS(1000));

	/* Reset I2C bus */
	#ifdef DEBUG_TOF
		ESP_LOGI("TOF", "Reseting I2C line");
	#endif
	init_gpio(GPIO_MODE_OUTPUT, GPIO_INTR_DISABLE, io->RST_pin, GPIO_PULLDOWN_DISABLE, GPIO_PULLUP_DISABLE);
	gpio_set(io->RST_pin);
	vTaskDelay(pdMS_TO_TICKS(500));
	
	gpio_set(io->LPn_pin);
	gpio_reset(io->RST_pin);
	vTaskDelay(pdMS_TO_TICKS(500));

	/* Disable communication */
}

uint8_t VL53LMZ_Init(VL53LMZ_Object* dev, I2C_Bus* bus, uint16_t address){
	/* Default configuration */

	#ifdef DEBUG_TOF
		ESP_LOGI("TOF", "Initializing...");
	#endif

	//dev->conf.platform.address = VL53LMZ_DEFAULT_I2C_ADDRESS;
	dev->conf.platform.Write = i2c_send_RS16;
	dev->conf.platform.Read = i2c_receive_RS16;

	/* I2C bus initialization */

	init_tof_i2c(bus, &dev->conf.platform);

	/* Enable communication for device */

    /* Check if I2C bus is busy */

	/* Check if device is available at selected address.
	 * Read firmware version on device and module type */
    uint8_t alive;
    uint8_t status = vl53lmz_is_alive(&dev->conf, &alive);

    /* Set new I2C address for device */
	// status = vl53lmz_set_i2c_address(&dev->conf, address);
	// if ( status != VL53LMZ_STATUS_OK){
	// 	return status;
	// }

	/* Upload new firmware to device */
	#ifdef DEBUG_TOF
		ESP_LOGI("TOF", "Uploading new firmware...");
	#endif
	status = vl53lmz_init(&dev->conf);
	if ( status != VL53LMZ_STATUS_OK){
		return status;
	}

	/* Disable communication so other devices can be set up */
	return status;
}

uint8_t VL53LMZ_Config(VL53LMZ_Configuration* conf, uint8_t resolution, uint8_t ranging_mode, uint32_t integration_time, uint8_t ranging_frequency, uint8_t sharpener){
	uint8_t status = VL53LMZ_STATUS_ERROR;

	/* Set resolution 4x4 or 8x8 */
	if (resolution == 0){
		return VL53LMZ_STATUS_ERROR;
	}

	status = vl53lmz_set_resolution(conf, resolution);
	if ( status != VL53LMZ_STATUS_OK){
		return status;
	}

	/* Set ranging mode Continuouse or Autonomous */
	if (ranging_mode == 0){
		return VL53LMZ_STATUS_ERROR;
	}

	status = vl53lmz_set_ranging_mode(conf, ranging_mode);
	if ( status != VL53LMZ_STATUS_OK){
		return status;
	}


	/* Set integration time for Autonomouse mode */
	if (integration_time == 0){
		return VL53LMZ_STATUS_ERROR;
	}

	status = vl53lmz_set_integration_time_ms(conf, integration_time);
	if ( status != VL53LMZ_STATUS_OK){
		return status;
	}

	/* Set ranging frequency, max 30Hz for 4x4, or max 15Hz for 8x8 */
	if (ranging_frequency == 0){
		return VL53LMZ_STATUS_ERROR;
	}

//	if (resolution == VL53LMZ_RESOLUTION_4X4 && ranging_frequency > 30){
//		ranging_frequency = 30;
//	} else if (resolution == VL53LMZ_RESOLUTION_8X8 && ranging_frequency > 15){
//		ranging_frequency = 15;
//	}

	status = vl53lmz_set_ranging_frequency_hz(conf, ranging_frequency);
	if ( status != VL53LMZ_STATUS_OK){
		return status;
	}


	/* Set sharpener percent */
	if (sharpener == 0){
		return VL53LMZ_STATUS_ERROR;
	}

	if (sharpener > 99){
		sharpener = 99;
	}

	status = vl53lmz_set_sharpener_percent(conf, sharpener);

	return status;
}

uint8_t VL53LMZ_Start_Ranging(VL53LMZ_Configuration* conf){
	uint8_t status = VL53LMZ_STATUS_ERROR;

	status = vl53lmz_start_ranging(conf);
	return status;
}

uint8_t VL53LMZ_Get_Distance(VL53LMZ_Configuration* conf, VL53LMZ_Result_t* result){
	uint8_t status = VL53LMZ_STATUS_ERROR;
	uint8_t new_data = 0;
	uint32_t tick_start = 0;
	//uint8_t resolution = 0;

	tick_start = conf->platform.GetTick();

	do
	{
	  status = vl53lmz_check_data_ready(conf, &new_data);

	  if (new_data == 1U)
	  {
		status = VL53LMZ_STATUS_OK;
		break;
	  }
	} while ((conf->platform.GetTick() - tick_start) < 5000);  // 5s timeout

	if (new_data == 0U){
		return VL53LMZ_STATUS_TIMEOUT_ERROR;
	}

//	status = vl53lmz_get_resolution(conf, &resolution);
//	if ( status != VL53LMZ_STATUS_OK){
//		return status;
//	}

	VL53LMZ_ResultsData raw_data;
	status = vl53lmz_get_ranging_data(conf, &raw_data);
	if ( status != VL53LMZ_STATUS_OK){
		return status;
	}

	VL53LMZ_Get_Result(raw_data, 64, result);

	return status;
}

//uint8_t VL53LMZ_Get_Distance_IT(VL53LMZ_Configuration* conf, VL53LMZ_ResultsData* data){
//
//}

void VL53LMZ_Get_Result(VL53LMZ_ResultsData raw, uint8_t resolution, VL53LMZ_Result_t* data){
	data->NumberOfZones = resolution;

	for (uint8_t i = 0; i < resolution; ++i)
	{
		data->ZoneResult[i].Distance = (uint32_t)raw.distance_mm[i];

		/* return Ambient value if ambient rate output is enabled */
#ifndef VL53LMZ_DISABLE_AMBIENT_PER_SPAD
		data->ZoneResult[i].Ambient = (float)raw.ambient_per_spad[i];
#else
		data->ZoneResult[i].Ambient = 0.0f;
#endif

		/* return Signal value if signal rate output is enabled */
#ifndef VL53LMZ_DISABLE_SIGNAL_PER_SPAD
		data->ZoneResult[i].Signal = (float)raw.signal_per_spad[i];
#else
		data->ZoneResult[i].Signal = 0.0f;
#endif

		/* Map target status */
		uint8_t target_status = raw.target_status[i];
		if ((target_status == 5U)|| (target_status == 9U)){
			data->ZoneResult[i].Status = 0U; /* ranging is OK */
	    } else if (target_status == 0U){
	    	data->ZoneResult[i].Status = 255U; /* no update */
	    } else {
	    	data->ZoneResult[i].Status = target_status; /* return device status otherwise */
	    }

	}
}



