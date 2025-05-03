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

VL53LMZ_Interrupt_Zone zone = {
    .interrupt_left = 0,
    .interrupt_right = 0,
    .interrupt_center = 0
};

Servo servo_right = {
    .max_duty = SERVO_MAX_DUTY_PERCENT,
    .min_duty = SERVO_MIN_DUTY_PERCENT,

    .angle_open = SERVO_ANGLE_ORT_INV,
    .angle_closed = SERVO_ANGLE_ZERO_INV
};

Servo servo_left = {
    .max_duty = SERVO_MAX_DUTY_PERCENT,
    .min_duty = SERVO_MIN_DUTY_PERCENT,

    .angle_open = SERVO_ANGLE_ORT,
    .angle_closed = SERVO_ANGLE_ZERO
};

void Error_Handler(){
    ESP_LOGE("Error", "error :)");
    while(1){
        vTaskDelay(pdMS_TO_TICKS(5000));

    }
}

void app_main(void)
{

    init_servo(&servo_right, GPIO_NUM_17, LEDC_TIMER_0, LEDC_CHANNEL_0, 50, 11.7, 3);
    init_servo(&servo_left, GPIO_NUM_18, LEDC_TIMER_1, LEDC_CHANNEL_1, 50, 11.7, 3);

    servo_move(&servo_right, servo_right.angle_closed);
    servo_move(&servo_left, servo_left.angle_closed);

    // servo_move(&servo_right, servo_right.angle_open);
    // servo_move(&servo_left, servo_left.angle_open);

    // init_wifi(WIFI_SSID, WIFI_PASS);
    // init_socket(&rpi_socket);

    // init_i2c0(GPIO_NUM_4, GPIO_NUM_5);

    // init_motor();

    /***
     * @todo Test connection again
     * socket_connect(&rpi_socket, SERVER_IP, SERVER_PORT);
     * send sima ID
     * receive data for sima (max 128 bytes)
     * convert received data to float
     */

    

    // VL53LMZ_Reset(&tof.io);
    // status = VL53LMZ_Init(&tof, tof.conf.platform.address);
    // if ( status != VL53LMZ_STATUS_OK ){
    //     Error_Handler();
    // }

    // status = VL53LMZ_Config(&tof.conf, TOF_NUMBER_OF_ZONES, VL53LMZ_RANGING_MODE_CONTINUOUS, 30, 15, 30);
    // if ( status != VL53LMZ_STATUS_OK ){
    //     Error_Handler();
    // }

    // status = vl53lmz_start_ranging(&tof.conf);
    // if ( status != VL53LMZ_STATUS_OK ){
  	//     Error_Handler();
    // }

    // init_tof_intr(tof.io.INTR_pin, &tof);
    // vTaskDelay(pdMS_TO_TICKS(500));
    
    while(1){  

        // servo_wave(&servo_right);
        // servo_wave(&servo_left);
        // vTaskDelay(pdMS_TO_TICKS(250));

        // servo_wave(&servo_right);
        // servo_wave(&servo_left);
        // vTaskDelay(pdMS_TO_TICKS(250));

        // if(get_tof_intr(&tof)){
        //     status = VL53LMZ_STATUS_OK;

        //     status |= VL53LMZ_Get_Distance(&tof.conf, &data);
        //     status |= ConvertDist2Point(&data, &tof, 450.0);

        //     tof_calculate_distances_interrupt(&tof, &data);

            // for(uint16_t i = 0; i < VL53LMZ_RESOLUTION_4X4 / 4; i++){
            //     ESP_LOGI("Distances:", "%lu %lu %lu %lu (%lu %lu %lu %lu)", 
            //                             data.ZoneResult[i * 4].Distance, data.ZoneResult[i * 4 +1].Distance, 
            //                             data.ZoneResult[i * 4+2].Distance, data.ZoneResult[i*4+3].Distance, 
            //                             data.ZoneResult[i * 4].Status, data.ZoneResult[i * 4 +1].Status, 
            //                             data.ZoneResult[i * 4+2].Status, data.ZoneResult[i*4+3].Status);
            // }        
        // }

        // if(get_tof_intr_zone(&tof, &data, &zone)){
        //     if(zone.interrupt_left){
        //         ESP_LOGI("Interrupt zone", "left");
        //     }

        //     if(zone.interrupt_right){
        //         ESP_LOGI("Interrupt zone", "right");
        //     }

        //     if(zone.interrupt_center){
        //         ESP_LOGI("Interrupt zone", "center");
        //     }
        // }

    }
}