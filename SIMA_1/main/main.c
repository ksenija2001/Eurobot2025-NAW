#include "main.h"

Socket rpi_socket = {
    .tag = "RPI"
};

static uint8_t status;

VL53LMZ_Result_t data;
VL53LMZ_Object tof = {
		.io = {
            .INTR_pin   = GPIO_NUM_10, 
			.LPn_pin    = GPIO_NUM_14,
			.RST_pin    = GPIO_NUM_11,  
			.PWR_EN_pin = GPIO_NUM_9
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

robot_states_e robot_states = ROBOT_WAIT_FOR_SIGNAL;

bool i2c_initialized = false;
bool robot_stopped = false;
bool in_position = false;

int right_count = 0;
int left_count = 0;
uint64_t timer_count = 0;

encoder_info_queue_t encoder;
QueueHandle_t queue;
position_t robot;
position_t temp_robot;

int i = 1;

int data_size;

motor_t right_motor;
motor_t left_motor;

pcnt_unit_handle_t pcnt1;
pcnt_unit_config_t pcnt1_conf;
pcnt_channel_handle_t pcnt1_chan;
pcnt_chan_config_t pcnt1_chan_conf;
pcnt_unit_handle_t pcnt2;
pcnt_unit_config_t pcnt2_conf;
pcnt_channel_handle_t pcnt2_chan;
pcnt_chan_config_t pcnt2_chan_conf;

void Error_Handler(){
    ESP_LOGE("Error", "error :)");
    while(1){
        vTaskDelay(pdMS_TO_TICKS(5000));

    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    if (i==data_size-2)
    {
        motors_stop(right_motor, left_motor);
        robot_states = ROBOT_SERVO;
    }
}

static bool IRAM_ATTR my_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
	pcnt_unit_get_count(pcnt1, &right_count);
	pcnt_unit_get_count(pcnt2, &left_count);
    encoder.left = left_count;
    encoder.right = right_count;
    //xQueueSendFromISR(queue, &encoder, &high_task_awoken);  //Zakomentarisi ovaj
	timer_count += 4;
    if (timer_count >= 14000)
    {
        motors_stop(right_motor, left_motor);
        robot_states = ROBOT_SERVO;
    }
    return (high_task_awoken == pdTRUE);
}

void app_main(void)
{
	queue = xQueueCreate(100, sizeof(encoder_info_queue_t));
	if (!queue)
	{
		printf("Queue nije dobro kreiran");
	}

    //xTaskCreatePinnedToCore(queue_read, "queue_read", 4096, NULL, 10, NULL, 1);

    #ifdef WIFI_USED
        coord_t coords[5];
    #else
        coord_t coords[] = { {125.0, 276.0, 0.0, 0.0}, { 250.0, 276.0, 0.0, 800.0 } };    //{125.0, 395.0, 0.0, 0.0}, { 1000.0, 600.0, 0.0, 50.0 }, { 1800.0, 580.0, 0.0, 50.0 }
        data_size = sizeof(coords)/16;
        //target_zone = set_target_zone(2);
        //ESP_LOGW("Zona", "XMIN: %f XMAX: %f YMIN: %f YMAX: %f", target_zone.x_min, target_zone.x_max, target_zone.y_min, target_zone.y_max);
    #endif
    // coord_t coords[] = { {125.0, 395.0, 0.0, 0.0}, { 500.0, 395.0, 0.0, 50.0 }, { 1000.0, 600.0, 0.0, 50.0 }, { 1800.0, 590.0, 0.0, 50.0 } };    //{125.0, 395.0, 0.0, 0.0}, { 1000.0, 600.0, 0.0, 50.0 }, { 1800.0, 580.0, 0.0, 50.0 }
    // data_size = sizeof(coords)/16;


    //WIFI
    #ifdef WIFI_USED
        init_wifi(WIFI_SSID, WIFI_PASS);
        init_socket(&rpi_socket);
        socket_connect(&rpi_socket, SERVER_IP, SERVER_PORT);
        strcpy(&rpi_socket.tx_buff, SIMA_ID);
        socket_send(&rpi_socket);
        socket_recv(&rpi_socket);   //Size of data
        ESP_LOGI("RPi", "%d", rpi_socket.rx_buff[0] + '0');
        ESP_LOGI("RPi", "Primio");
        data_size = (int)rpi_socket.rx_buff[0];
        //target_zone = set_target_zone((int)rpi_socket.rx_buff[1]);   //NE ZABORAVI NA OVO
        for (int i=0; i<data_size; i++)
        {
            memcpy(&coords[i], &rpi_socket.rx_buff[i*16+1], sizeof(coord_t));
        }
    #endif
    //-----------------------------------------------------------------------------------------------------------------------------

    //I2C
    #ifdef DETECTION
        init_i2c0(GPIO_NUM_13, GPIO_NUM_12);

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
        vTaskDelay(pdMS_TO_TICKS(500));

        xTaskCreatePinnedToCore(app_main_2, "app_main_2", 4096, NULL, 5, NULL, 1);
    #endif
    #ifndef DETECTION
        i2c_initialized = 1;
    #endif
    //-------------------------------------------------------

    //MICROSWITCH
    //gpio_config_t microswitch;
    //microswitch_init(&microswitch, GPIO_INTR_POSEDGE, GPIO_IN, MICROSWITCH, GPIO_PU_DIS, GPIO_PD_DIS);
    //microswitch_interrupt(0, MICROSWITCH, gpio_isr_handler);
    //-----------------------------------------------------------------------------------------------------------------------------

    //MOTORS
    mcpwm_config_t motor_config;
    mcpwm_config_t servo_config;
    motor_t servo1;
    motor_t servo2;

    mcpwm_unit_setup(&motor_config, MCPWM_UP_COUNTER, 500);
	motor_init(&right_motor, &motor_config, UNIT_0, TIMER_0, MR_A, MR_B);
	motor_init(&left_motor, &motor_config, UNIT_0, TIMER_1, ML_A, ML_B);

    mcpwm_unit_setup(&servo_config, MCPWM_UP_COUNTER, 50);
    motor_init(&servo1, &servo_config, UNIT_1, TIMER_0, SERVO_1, NULL);
    motor_init(&servo2, &servo_config, UNIT_1, TIMER_1, SERVO_2, NULL);
    //-------------------------------------------------------------------------------------------------------------------------------

    //PULSE COUNTER
    pcnt_unit_init(&pcnt1, &pcnt1_conf, 32000, -32000, 1);
    pcnt_unit_init(&pcnt2, &pcnt2_conf, 32000, -32000, 1);
    pcnt_channel_init(pcnt1, &pcnt1_chan_conf, &pcnt1_chan, RIGHT_ENC_EDGE, RIGHT_ENC_LEVEL);
    pcnt_channel_init(pcnt2, &pcnt2_chan_conf, &pcnt2_chan, LEFT_ENC_EDGE, LEFT_ENC_LEVEL);
    pcnt_set_pin_actions(pcnt1_chan);
    pcnt_set_pin_actions(pcnt2_chan);
    pcnt_enable_clear_start(pcnt1);
    pcnt_enable_clear_start(pcnt2);
    //---------------------------------------------------------------------------------------------------------------------------------

    //GPTIMER
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t gptimer_conf;
    gptimer_alarm_config_t gptimer_alarm;
    gptimer_event_callbacks_t cbs;
    gptimer_init(&gptimer, &gptimer_conf, GPTIMER_CLK_SRC_DEFAULT, GPTIMER_COUNT_UP, 1000000, 0);
    gptimer_alarm_init(gptimer, &gptimer_alarm, 4000, 0, 1);
    gptimer_cbs_init(gptimer, &cbs, my_callback, queue);    //Note to yourself: QueueHandle_t is a pointer of type void *. If you pass &queue, you are passing an address of the QueueHandle_t queue instead of the queue itself. ISR runs into a problem because it expects a void * pointer (which user_data parameter is)
                                                                    //You can either pass queue or (void *)queue
    if (robot_states == ROBOT_MOVE) gptimer_enable_and_start(gptimer);
    //-----------------------------------------------------------------------------------------------------------------------------------

    odom_init(WHEEL_CIRC, WHEEL_DIFF, CALIBRATION, &robot);
    set_start_position(coords[0].x, coords[0].y, coords[0].teta, &robot);

    esp_log_level_set(TAG_Control, ESP_LOG_INFO);
    esp_log_level_set(TAG_Control, ESP_LOG_WARN);

    while (1)
    {
        switch(robot_states)
        {
            case ROBOT_WAIT_FOR_SIGNAL:
                #ifdef WIFI_USED
                    ESP_LOGI("WIFI", "Cekamo");
                    socket_recv(&rpi_socket);
                    if ((int)rpi_socket.rx_buff[0] == 1 && i2c_initialized) 
                    {
                        robot_states = ROBOT_MOVE;
                        gptimer_enable_and_start(gptimer);
                        vTaskDelay(2000/portTICK_PERIOD_MS);
                    }
                #else
                    if (i2c_initialized) 
                    {
                        robot_states = ROBOT_MOVE;
                        gptimer_enable_and_start(gptimer);
                        vTaskDelay(2000/portTICK_PERIOD_MS);
                    }
                #endif
            break;
            case ROBOT_MOVE:
                go_to_xy(coords[i].x, coords[i].y, coords[i].speed, &encoder, &robot, right_motor, left_motor, pcnt1, pcnt2);
                if (finished_movement) robot_states = ROBOT_CHECK;
            break;
            case ROBOT_CHECK:
                i++;
                if (i<data_size) robot_states = ROBOT_MOVE;
                else 
                {
                    in_position = true;
                    robot_states = ROBOT_STOPPED;
                }
            break;
            case ROBOT_SERVO:
                servo_wave(servo1, servo2, 80.0, 30.0, 100.0, 150.0);
                vTaskDelay(10/portTICK_PERIOD_MS);
            break;
            case ROBOT_AVOID:
                //ESP_LOGI("State", "U robot avoid stejtu");
                motors_stop(right_motor, left_motor);
            break;
            case ROBOT_STOPPED:
                motors_stop(right_motor, left_motor);
                robot_stopped = true;
            break;
        }
        #ifdef DETECTION
            if (check_tof(data, zone, rpi_socket) && robot_states == ROBOT_MOVE && robot.x_pos >= 1000.0)
            {
                store_position(&robot, &temp_robot);
                disable_all_pcnts(pcnt1, pcnt2);
                encoder_reset(&encoder);
                robot_states = ROBOT_STOPPED;
            }
            else if (!check_tof(data, zone, rpi_socket) && robot_stopped && !in_position) 
            {
                robot_stopped = false;
                restore_position(&robot, &temp_robot);
                encoder_reset(&encoder);
                pcnt_enable_clear_start(pcnt1);
                pcnt_enable_clear_start(pcnt2);
                robot_states = ROBOT_MOVE;
            }
        #endif
        
        vTaskDelay(4/portTICK_PERIOD_MS);
    }
}

void app_main_2(){
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
    vTaskDelay(pdMS_TO_TICKS(500));
    i2c_initialized = true;

    while(1){
        if(get_tof_intr(&tof)){
            status = VL53LMZ_STATUS_OK;

            status |= VL53LMZ_Get_Distance(&tof.conf, &data);
            status |= ConvertDist2Point(&data, &tof, 450.0);

            tof_calculate_distances_interrupt(&tof, &data);

            // for(uint16_t i = 0; i < VL53LMZ_RESOLUTION_4X4 / 4; i++){
            //     ESP_LOGI("Distances:", "%lu %lu %lu %lu (%lu %lu %lu %lu)", 
            //                             data.ZoneResult[i * 4].Distance, data.ZoneResult[i * 4 +1].Distance, 
            //                             data.ZoneResult[i * 4+2].Distance, data.ZoneResult[i*4+3].Distance, 
            //                             data.ZoneResult[i * 4].Status, data.ZoneResult[i * 4 +1].Status, 
            //                             data.ZoneResult[i * 4+2].Status, data.ZoneResult[i*4+3].Status);
            // }        
        }

        if(get_tof_intr_zone(&tof, &data, &zone)){
            if(zone.interrupt_left){

                ESP_LOGI("Interrupt zone", "left");
            }

            if(zone.interrupt_right){
                ESP_LOGI("Interrupt zone", "right");
            }

            if(zone.interrupt_center){
                ESP_LOGI("Interrupt zone", "center");
            }
        }
    }
}

void store_position(position_t *robot, position_t *temp_robot)
{
    temp_robot->x_pos = robot->x_pos;
    temp_robot->y_pos = robot->y_pos;
    temp_robot->ang = robot->ang;
    temp_robot->angle_in_rad = robot->angle_in_rad;
    temp_robot->angle_in_deg = robot->angle_in_deg;
}

void restore_position(position_t *robot, position_t *temp_robot)
{
    robot->x_pos = temp_robot->x_pos;
    robot->y_pos = temp_robot->y_pos;
    robot->ang = temp_robot->ang;
    robot->angle_in_deg = temp_robot->angle_in_deg;
    robot->angle_in_rad = temp_robot->angle_in_rad;
}

