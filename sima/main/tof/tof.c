#include "tof.h"

void tof_sendByte(TOF* tof, uint16_t address, uint8_t data){
    tof_send(tof, address, &data, 1);
}

void tof_send(TOF* tof, uint16_t address, uint8_t* data, uint16_t size){

}

void tof_receive(TOF* tof, uint16_t address, uint8_t* data, uint16_t size){

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

uint8_t tof_poll_for_answer(
        TOF*                    tof, 
        uint8_t					size,
		uint8_t					pos,
		uint16_t				address,
		uint8_t					mask,
		uint8_t					expected_value)
{
    uint8_t buff[(uint16_t)(-1)] = {0};
    uint8_t timeout = 0;
    do {
        tof_receive(tof, address, buff, size);
		vTaskDelay(pdMS_TO_TICKS(10));

		if(timeout >= (uint8_t)200)	/* 2s timeout */
		{
            return 1;
			break;
		}else if((size >= (uint8_t)4)	
				&& (buff[2] >= (uint8_t)0x7f))
		{
            return 1;
			break;
		}
		else
		{
			timeout++;
		}
	}while ((buff[pos] & mask) != expected_value);

    return 0;
}

uint8_t tof_poll_for_mcu_boot(TOF* tof){
    uint8_t go2_status0, go2_status1;
	uint16_t timeout = 0;

    go2_status0 = go2_status1 = 0;

	do {
		tof_receive(tof, 0x06, &go2_status0, 1);
		if((go2_status0 & (uint8_t)0x80) != (uint8_t)0){
			
            tof_receive(tof, 0x07, &go2_status1, 1);
			if((go2_status1 & (uint8_t)0x01) != (uint8_t)0x00)
			{
				return 0;
				break;
			}
		}
        vTaskDelay(pdMS_TO_TICKS(1));
		timeout++;

		if((go2_status0 & (uint8_t)0x1) != (uint8_t)0){
			break;
		}
	}while (timeout < (uint16_t)500);

	return 1;
}

uint8_t tof_send_offset_data(TOF* tof, uint8_t resolution){
    uint8_t status = 0;
	uint32_t signal_grid[64];
	int16_t range_grid[64];
	uint8_t dss_4x4[] = {0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07};
	uint8_t footer[] = {0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4};
	int8_t i, j;
	uint16_t k;

	(void)memcpy(tof->temp_buffer, tof->offset_data, 488);

	/* Data extrapolation is required for 4X4 offset */
	if(resolution == (uint8_t)16){
		(void)memcpy(&(tof->temp_buffer[0x10]), dss_4x4, sizeof(dss_4x4));
		SwapBuffer(tof->temp_buffer, 488);
		(void)memcpy(signal_grid,&(tof->temp_buffer[0x3C]),
			sizeof(signal_grid));
		(void)memcpy(range_grid,&(tof->temp_buffer[0x140]),
			sizeof(range_grid));

		for (j = 0; j < (int8_t)4; j++)
		{
			for (i = 0; i < (int8_t)4 ; i++)
			{
				signal_grid[i+(4*j)] =
				(signal_grid[(2*i)+(16*j)+ (int8_t)0]
				+ signal_grid[(2*i)+(16*j)+(int8_t)1]
				+ signal_grid[(2*i)+(16*j)+(int8_t)8]
				+ signal_grid[(2*i)+(16*j)+(int8_t)9])
								  /(uint32_t)4;
				range_grid[i+(4*j)] =
				(range_grid[(2*i)+(16*j)]
				+ range_grid[(2*i)+(16*j)+1]
				+ range_grid[(2*i)+(16*j)+8]
				+ range_grid[(2*i)+(16*j)+9])
								  /(int16_t)4;
			}
		}
		(void)memset(&range_grid[0x10], 0, (uint16_t)96);
		(void)memset(&signal_grid[0x10], 0, (uint16_t)192);
		(void)memcpy(&(tof->temp_buffer[0x3C]),
					signal_grid, sizeof(signal_grid));
		(void)memcpy(&(tof->temp_buffer[0x140]),
					range_grid, sizeof(range_grid));
		SwapBuffer(tof->temp_buffer, 488);
	}

	for(k = 0; k < (488 - (uint16_t)4); k++)
	{
		tof->temp_buffer[k] = tof->temp_buffer[k + (uint16_t)8];
	}

	(void)memcpy(&(tof->temp_buffer[0x1E0]), footer, 8);
    tof_send(tof, 0x2e18, tof->temp_buffer, 488);
    status |= tof_poll_for_answer(tof, 4, 1, 0x2c00, 0xff, 0x03);
	
	return status;
}

uint8_t tof_send_xtalk_data(TOF* tof, uint8_t resolution){
    uint8_t status = 0;
	uint8_t res4x4[] = {0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07};
	uint8_t dss_4x4[] = {0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08};
	uint8_t profile_4x4[] = {0xA0, 0xFC, 0x01, 0x00};
	uint32_t signal_grid[64];
	int8_t i, j;

	(void)memcpy(tof->temp_buffer, &(tof->xtalk_data[0]),
		776);

	/* Data extrapolation is required for 4X4 Xtalk */
	if(resolution == (uint8_t)16)
	{
		(void)memcpy(&(tof->temp_buffer[0x8]),
			res4x4, sizeof(res4x4));
		(void)memcpy(&(tof->temp_buffer[0x020]),
			dss_4x4, sizeof(dss_4x4));

		SwapBuffer(tof->temp_buffer, 776);
		(void)memcpy(signal_grid, &(tof->temp_buffer[0x34]),
			sizeof(signal_grid));

		for (j = 0; j < (int8_t)4; j++)
		{
			for (i = 0; i < (int8_t)4 ; i++)
			{
				signal_grid[i+(4*j)] =
				(signal_grid[(2*i)+(16*j)+0]
				+ signal_grid[(2*i)+(16*j)+1]
				+ signal_grid[(2*i)+(16*j)+8]
				+ signal_grid[(2*i)+(16*j)+9])/(uint32_t)4;
			}
		}
		(void)memset(&signal_grid[0x10], 0, (uint32_t)192);
		(void)memcpy(&(tof->temp_buffer[0x34]),
				  signal_grid, sizeof(signal_grid));
		SwapBuffer(tof->temp_buffer, 776);
		(void)memcpy(&(tof->temp_buffer[0x134]),
		profile_4x4, sizeof(profile_4x4));
		(void)memset(&(tof->temp_buffer[0x078]),0 ,
						 (uint32_t)4*sizeof(uint8_t));
	}

    tof_send(tof, 0x2cf8, tof->temp_buffer, 776);
	tof_poll_for_answer(tof, 4, 1, 0x2c00, 0xff, 0x03);
	
	return status;
}

uint8_t tof_dci_write_data(TOF* tof, uint8_t* data, uint32_t index, uint16_t data_size){
    uint8_t status = 0;
	int16_t i;

	uint8_t headers[] = {0x00, 0x00, 0x00, 0x00};
	uint8_t footer[] = {0x00, 0x00, 0x00, 0x0f, 0x05, 0x01,
			(uint8_t)((data_size + (uint16_t)8) >> 8), 
			(uint8_t)((data_size + (uint16_t)8) & (uint8_t)0xFF)};

	uint16_t address = (uint16_t)0x2fff -
		(data_size + (uint16_t)12) + (uint16_t)1;

	/* Check if cmd buffer is large enough */
	if((data_size + (uint16_t)12) 
		   > (uint16_t)6688)
	{
		status |= 1;
	}
	else
	{
		headers[0] = (uint8_t)(index >> 8);
		headers[1] = (uint8_t)(index & (uint32_t)0xff);
		headers[2] = (uint8_t)(((data_size & (uint16_t)0xff0) >> 4));
		headers[3] = (uint8_t)((data_size & (uint16_t)0xf) << 4);

	/* Copy data from structure to FW format (+4 bytes to add header) */
		SwapBuffer(data, data_size);
		for(i = (int16_t)data_size - (int16_t)1 ; i >= 0; i--)
		{
			tof->temp_buffer[i + 4] = data[i];
		}

	/* Add headers and footer */
		(void)memcpy(&tof->temp_buffer[0], headers, sizeof(headers));
		(void)memcpy(&tof->temp_buffer[data_size + (uint16_t)4],
			footer, sizeof(footer));

	/* Send data to FW */
        tof_send(tof, address, tof->temp_buffer, (uint32_t)((uint32_t)data_size + (uint32_t)12));
        tof_poll_for_answer(tof, 4, 1, 0x2c00, 0xff, 0x03);
            
        SwapBuffer(data, data_size);
	}

	return status;
}

uint8_t tof_dci_read_data(TOF* tof, 
        uint8_t				*data,
		uint32_t			index,
		uint16_t			data_size)
{
    int16_t i;
	uint8_t status = 0;
		uint32_t rd_size = (uint32_t) data_size + (uint32_t)12;
	uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x0f,
			0x00, 0x02, 0x00, 0x08};

	/* Check if tmp buffer is large enough */
	if((data_size + (uint16_t)12)>(uint16_t)6688)
	{
		status |= 1;
	}
	else
	{
		cmd[0] = (uint8_t)(index >> 8);	
		cmd[1] = (uint8_t)(index & (uint32_t)0xff);			
		cmd[2] = (uint8_t)((data_size & (uint16_t)0xff0) >> 4);
		cmd[3] = (uint8_t)((data_size & (uint16_t)0xf) << 4);

	/* Request data reading from FW */
    tof_send(tof, 0x2fff - (uint16_t)11, cmd, sizeof(cmd));
	tof_poll_for_answer(tof, 4, 1, 0x2c00, 0xff, 0x03);

	/* Read new data sent (4 bytes header + data_size + 8 bytes footer) */
    tof_receive(tof, 0x2c04, tof->temp_buffer, rd_size);
	SwapBuffer(tof->temp_buffer, data_size + (uint16_t)12);

	/* Copy data from FW into input structure (-4 bytes to remove header) */
		for(i = 0 ; i < (int16_t)data_size;i++){
			data[i] = tof->temp_buffer[i + 4];
		}
	}

	return status;
}

uint8_t tof_dci_replace_data(TOF* tof, 
        uint8_t				*data,
		uint32_t			index,
		uint16_t			data_size,
		uint8_t				*new_data,
		uint16_t			new_data_size,
		uint16_t			new_data_pos)
{
    uint8_t status = 0;

    tof_dci_read_data(tof, data, index, data_size);
	(void)memcpy(&(data[new_data_pos]), new_data, new_data_size);
	tof_dci_write_data(tof, data, index, data_size);

	return status;
}

void init_tof(TOF* tof, I2C_Bus* bus, uint8_t address, uint8_t rst){

#if defined(DEBUG_TOF)
    ESP_LOGI(tof->tag, "Initializing TOF with address 0x%x", address);
#endif

    tof->bus = bus;
    tof->speed = 400000;
    
    init_i2c_device(bus, &(tof->device), address, tof->speed);
    init_gpio(GPIO_MODE_OUTPUT, GPIO_INTR_DISABLE, GPIO_NUM_6, GPIO_PULLDOWN_DISABLE, GPIO_PULLUP_DISABLE);

    tof->i2c_rst_pin = rst;
    tof->address = address;

#if defined(DEBUG_TOF)
    ESP_LOGI(tof->tag, "Reseting I2C driver");
#endif

    gpio_reset(tof->i2c_rst_pin);
    vTaskDelay(pdMS_TO_TICKS(500));

    gpio_set(tof->i2c_rst_pin);
    vTaskDelay(pdMS_TO_TICKS(500));

    gpio_reset(tof->i2c_rst_pin);
    vTaskDelay(pdMS_TO_TICKS(500));

    tof_is_alive(tof);
    //tof_configure(tof);
}

void tof_is_alive(TOF* tof){
    esp_err_t err = i2c_device_alive(tof->bus, tof->address);
    
    if(err == ESP_OK){
        #if defined(DEBUG_TOF)
            ESP_LOGI(tof->tag, "alive");
        #endif

        uint8_t data[2] = {254, 254};

        tof_sendByte(tof, 0x7fff, 0x00);
        tof_receive(tof, 0x0000, data, 2);
        tof_sendByte(tof, 0x7fff, 0x02);

        #if defined(DEBUG_TOF)
            ESP_LOGI(tof->tag, "Device ID: %d # Revision ID: %d", data[0], data[1]);
        #endif

    }else{
        #if defined(DEBUG_TOF)
            ESP_LOGE(tof->tag, "not alive");
        #endif
    }
}

// void tof_firmware(TOF* tof){

//     uint8_t tmp;
//     uint8_t buff[(uint16_t)(-1)];
//     uint8_t pipe_ctrl[] = {0x01, 0x00, 0x01, 0x00};
//     uint32_t single_range = 0x01;

//     /* SW reboot sequence */
// 	tof_sendByte(tof, 0x7fff, 0x00);
// 	tof_sendByte(tof, 0x0009, 0x04);
// 	tof_sendByte(tof, 0x000F, 0x40);
// 	tof_sendByte(tof, 0x000A, 0x03);
// 	tof_receive(tof, 0x7FFF, &tmp, 1);
// 	tof_sendByte(tof, 0x000C, 0x01);

// 	tof_sendByte(tof, 0x0101, 0x00);
// 	tof_sendByte(tof, 0x0102, 0x00);
// 	tof_sendByte(tof, 0x010A, 0x01);
// 	tof_sendByte(tof, 0x4002, 0x01);
// 	tof_sendByte(tof, 0x4002, 0x00);
// 	tof_sendByte(tof, 0x010A, 0x03);
// 	tof_sendByte(tof, 0x0103, 0x01);
// 	tof_sendByte(tof, 0x000C, 0x00);
// 	tof_sendByte(tof, 0x000F, 0x43);
//     vTaskDelay(pdMS_TO_TICKS(1));

// 	tof_sendByte(tof, 0x000F, 0x40);
// 	tof_sendByte(tof, 0x000A, 0x01);
//     vTaskDelay(pdMS_TO_TICKS(100));
	
//     /* Wait for sensor booted (several ms required to get sensor ready ) */
// 	tof_sendByte(tof, 0x7fff, 0x00);
// 	if(tof_poll_for_answer(tof, 1, 0, 0x06, 0xff, 1) != 0){
// 		// goto exit;
// 		return;
// 	}

// 	tof_sendByte(tof, 0x000E, 0x01);

// 	/* Enable FW access */

//     tof_sendByte(tof, 0x7fff, 0x02);
//     tof_sendByte(tof, 0x03, 0x0D);
//     tof_sendByte(tof, 0x7fff, 0x01);
//     tof_poll_for_answer(tof, 1, 0, 0x21, 0x10, 0x10);
	
// 	tof_sendByte(tof, 0x7fff, 0x00);

// 	/* Enable host access to GO1 */
// 	tof_receive(tof, 0x7fff, &tmp, 1);
// 	tof_sendByte(tof, 0x0C, 0x01);

// 	/* Power ON status */
// 	tof_sendByte(tof, 0x7fff, 0x00);
// 	tof_sendByte(tof, 0x101, 0x00);
// 	tof_sendByte(tof, 0x102, 0x00);
// 	tof_sendByte(tof, 0x010A, 0x01);
// 	tof_sendByte(tof, 0x4002, 0x01);
// 	tof_sendByte(tof, 0x4002, 0x00);
// 	tof_sendByte(tof, 0x010A, 0x03);
// 	tof_sendByte(tof, 0x103, 0x01);
// 	tof_sendByte(tof, 0x400F, 0x00);
// 	tof_sendByte(tof, 0x21A, 0x43);
// 	tof_sendByte(tof, 0x21A, 0x03);
// 	tof_sendByte(tof, 0x21A, 0x01);
// 	tof_sendByte(tof, 0x21A, 0x00);
// 	tof_sendByte(tof, 0x219, 0x00);
// 	tof_sendByte(tof, 0x21B, 0x00);

// 	/* Wake up MCU */
// 	tof_sendByte(tof, 0x7fff, 0x00);
// 	tof_receive(tof, 0x7fff, &tmp, 1);
// 	tof_sendByte(tof, 0x0C, 0x00);
// 	tof_sendByte(tof, 0x7fff, 0x01);
// 	tof_sendByte(tof, 0x20, 0x07);
// 	tof_sendByte(tof, 0x20, 0x06);

// 	/* Download FW into VL53LMZ */
// 	tof_sendByte(tof, 0x7fff, 0x09);
// 	tof_send(tof, 0, (uint8_t*)&VL53LMZ_FIRMWARE[0], 0x8000);
	
//     tof_sendByte(tof, 0, 0x0a);
//     tof_send(tof, 0, (uint8_t*)&VL53LMZ_FIRMWARE[0x8000], 0x8000);

// 	tof_sendByte(tof, 0x7fff, 0x0b);
//     tof_send(tof, 0, (uint8_t*)&VL53LMZ_FIRMWARE[0x10000], 0x5000);
// 	tof_sendByte(tof, 0x7fff, 0x01);

//     tof_sendByte(tof, 0x7fff, 0x02);
//     tof_sendByte(tof, 0x03, 0x0D);
//     tof_sendByte(tof, 0x7fff, 0x01);
    
// 	/* Check if FW correctly downloaded */
// 	if(tof_poll_for_answer(tof, 1, 0, 0x21, 0x10, 0x10) != 0) {
// 		// goto exit;
// 		return;
// 	}

// 	tof_sendByte(tof, 0x7fff, 0x00);
// 	tof_receive(tof, 0x7fff, &tmp, 1);
// 	tof_sendByte(tof, 0x0C, 0x01);

// 	/* Reset MCU and wait boot */
// 	tof_sendByte(tof, 0x7FFF, 0x00);
// 	tof_sendByte(tof, 0x114, 0x00);
// 	tof_sendByte(tof, 0x115, 0x00);
// 	tof_sendByte(tof, 0x116, 0x42);
// 	tof_sendByte(tof, 0x117, 0x00);
// 	tof_sendByte(tof, 0x0B, 0x00);
// 	tof_receive(tof, 0x7fff, &tmp, 1);
// 	tof_sendByte(tof, 0x0C, 0x00);
// 	tof_sendByte(tof, 0x0B, 0x01);

// 	if(tof_poll_for_mcu_boot(tof) != 0){
// 		// goto exit;
// 		return;
// 	}

// 	tof_sendByte(tof, 0x7fff, 0x02);

// 	/* Get offset NVM data and store them into the offset buffer */
//     tof_send(tof, 0x2fd8, (uint8_t*)VL53LMZ_GET_NVM_CMD, sizeof(VL53LMZ_GET_NVM_CMD));
// 	tof_poll_for_answer(tof, 4, 0, 0x2c00, 0xff, 2);
	
//     tof_receive(tof, 0x2c04, buff, 492);
	
// 	(void)memcpy(tof->offset_data, buff, 488);
//     tof_send_offset_data(tof, 16);

// 	/* Set default Xtalk shape. Send Xtalk to sensor */

// 	tof->default_xtalk = (uint8_t*)VL53LMZ_DEFAULT_XTALK;
// 	(void)memcpy(tof->xtalk_data, (uint8_t*)VL53LMZ_DEFAULT_XTALK, 776);
//     tof_send_xtalk_data(tof, 16);

// 	/* Send default configuration to VL53L5CX firmware */

//     tof->default_configuration = (uint8_t*)VL53L7_DEFAULT_CONFIGURATION;
//     tof_send(tof, 0x2c34, tof->default_configuration, sizeof(VL53L7_DEFAULT_CONFIGURATION));
//     tof_poll_for_answer(tof, 4, 1, 0x2c00, 0xff, 0x03);

//     tof_dci_write_data(tof, (uint8_t*)&pipe_ctrl, 0xdb80, (uint16_t)sizeof(pipe_ctrl));

//     tof_dci_write_data(tof, (uint8_t*)&single_range, 0xd964, (uint16_t)sizeof(single_range));
	
// 	tmp = (uint8_t)1;

//     tof_dci_replace_data(tof, tof->temp_buffer, 0xe108, 40, (uint8_t*)&tmp, 1, 0x26);
//     tof_dci_replace_data(tof, tof->temp_buffer, 0xe108, 40, (uint8_t*)&tmp, 1, 0x25);
	
// // exit:
// // 	return;
// 	return;
// }