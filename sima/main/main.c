#include "main.h"

static uint8_t status;

Socket rpi_socket = {
    .tag = "RPi"
};

VL53LMZ_Result_t data;
VL53LMZ_Object tof = {
		.io = {
            .INTR_pin   = GPIO_NUM_16,
			.LPn_pin    = GPIO_NUM_15,
			.RST_pin    = GPIO_NUM_6,  
			.PWR_EN_pin = GPIO_NUM_7
		},
        .conf = {
            .platform = {
                .address = I2C_TOF_ADDRESS,
                .speed = 400000
            }
        },
		.trans_offset = { .vector = {0.0, 282.0, 0.0} },
		.orient_offset = { .vector = {0} }
};

void Error_Handler(){
    ESP_LOGE("Error", "error :)");
    while(1){
        vTaskDelay(pdMS_TO_TICKS(5000));

    }
}

void app_main(void)
{
    // init_wifi(WIFI_SSID, WIFI_PASS);
    // init_socket(&rpi_socket);

    init_i2c0(GPIO_NUM_4, GPIO_NUM_5);

    // init_motor();

    /***
     * @todo Test connection again
     * socket_connect(&rpi_socket, SERVER_IP, SERVER_PORT);
     * send sima ID
     * receive data for sima (max 128 bytes)
     * convert received data to float
     */

    

    VL53LMZ_Reset(&tof.io);
    status = VL53LMZ_Init(&tof, tof.conf.platform.address);
    if ( status != VL53LMZ_STATUS_OK ){
        Error_Handler();
    }

    status = VL53LMZ_Config(&tof.conf, TOF_NUMBER_OF_ZONES, VL53LMZ_RANGING_MODE_CONTINUOUS, 30, 15, 30);
    if ( status != VL53LMZ_STATUS_OK ){
        Error_Handler();
    }

    status = vl53lmz_start_ranging(&tof.conf);
    if ( status != VL53LMZ_STATUS_OK ){
  	    Error_Handler();
    }

    init_tof_intr(tof.io.INTR_pin, &tof);
    
    while(1){  

        if(get_tof_intr(&tof)){
            status = VL53LMZ_STATUS_OK;

            status |= VL53LMZ_Get_Distance(&tof.conf, &data);
            status |= ConvertDist2Point(&data, &tof, 450.0);

            for(uint16_t i = 0; i < TOF_NUMBER_OF_ZONES / 4; i++){
                ESP_LOGI("Distances:", "%lu %lu %lu %lu (%lu %lu %lu %lu)", 
                                        data.ZoneResult[i * 4].Distance, data.ZoneResult[i * 4 +1].Distance, 
                                        data.ZoneResult[i * 4+2].Distance, data.ZoneResult[i*4+3].Distance, 
                                        data.ZoneResult[i * 4].Status, data.ZoneResult[i * 4 +1].Status, 
                                        data.ZoneResult[i * 4+2].Status, data.ZoneResult[i*4+3].Status);
            }        
        }
    }
}
