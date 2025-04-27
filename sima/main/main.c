#include "main.h"

Socket rpi_socket = {
    .tag = "RPi"
};

VL53LMZ_Result_t data;
VL53LMZ_Object tof = {
		.io = {
			.LPn_pin = GPIO_NUM_15,
			.RST_pin = GPIO_NUM_6,  
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

uint8_t status;

void Error_Handler(){
    ESP_LOGE("Error", "error :)");
    while(1){
        vTaskDelay(pdMS_TO_TICKS(5000));

    }
}

void app_main(void)
{
    init_i2c0(GPIO_NUM_4, GPIO_NUM_5);

    VL53LMZ_Reset(&tof.io);
    status = VL53LMZ_Init(&tof, tof.conf.platform.address);
    if ( status != VL53LMZ_STATUS_OK ){
        Error_Handler();
    }

    status = VL53LMZ_Config(&tof.conf, VL53LMZ_RESOLUTION_4X4, VL53LMZ_RANGING_MODE_CONTINUOUS, 30, 15, 30);
    if ( status != VL53LMZ_STATUS_OK ){
        Error_Handler();
    }

    status = vl53lmz_start_ranging(&tof.conf);
    if ( status != VL53LMZ_STATUS_OK ){
  	    Error_Handler();
    }

    // init_wifi(WIFI_SSID, WIFI_PASS);
    // init_socket(&rpi_socket);

    // socket_connect(&rpi_socket, SERVER_IP, SERVER_PORT);

    // socket_recv(&rpi_socket);
    // socket_close(&rpi_socket);

    // if(!strcmp("Sima 1", rpi_socket.rx_buff)){
    //     ESP_LOGE(rpi_socket.tag, "Running aplication 1: %s", rpi_socket.rx_buff);
    // }else{
    //     ESP_LOGE(rpi_socket.tag, "Error receiving sima code: %s", rpi_socket.rx_buff);
    // }
    
    while(1){  
        ESP_LOGI("App", "Running...");
        status = VL53LMZ_STATUS_OK;

        status |= VL53LMZ_Get_Distance(&tof.conf, &data);
        status |= ConvertDist2Point(&data, &tof, 450.0);

        for(uint16_t i = 0; i < VL53LMZ_RESOLUTION_4X4 / 4; i++){
            ESP_LOGI("Distances:", "%lu %lu %lu %lu", data.ZoneResult[i * 4].Distance, data.ZoneResult[i * 4 +1].Distance, data.ZoneResult[i * 4+2].Distance, data.ZoneResult[i*4+3].Distance);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
