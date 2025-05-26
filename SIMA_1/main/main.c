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

volatile VL53LMZ_Interrupt_Zone zone = {
    .interrupt_left = 0,
    .interrupt_right = 0,
    .interrupt_center = 0
};

// #ifdef DETECTION || WIFI_USED
//     robot_states_e robot_states = ROBOT_WAIT_FOR_SIGNAL;
// #else
//     robot_states_e robot_states = ROBOT_MOVE;
// #endif
// bool i2c_initialized = false;
// bool in_position = false;

sima_t sima;
spline_targets_t spline_targets;
volatile encoder_info_queue_t encoder;
QueueHandle_t queue;
position_t robot;
position_t temp_robot;
int i = 1;
int data_size;

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
        motors_stop(sima.right_motor, sima.left_motor);
        sima.robot_states = ROBOT_SERVO;
    }
}

static bool IRAM_ATTR my_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
	// pcnt_unit_get_count(pcnt1, &encoder.right);
	// pcnt_unit_get_count(pcnt2, &encoder.left);
    //xQueueSendFromISR(queue, &encoder, &high_task_awoken);  //Zakomentarisi ovaj
	sima.timer_count += 10;
    if (sima.timer_count >= 14000)
    {
        motors_stop(sima.right_motor, sima.left_motor);
        sima.robot_states = ROBOT_SERVO;
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

    #ifdef WIFI_USED
        coord_t coords[5];
    #else
        coord_t coords[] = { { 2875.0, 1845.0, 3.14, 0.0 }, { 2400, 1600.0, -2.4, 400.0 }, { 1900.0, 1450.0, 0.0, 400.0 } };    //125.0, 395.0, 1000, 600
        data_size = sizeof(coords)/16;
        sima.target_zone = set_target_zone(2);
        //ESP_LOGW("Zona", "XMIN: %f XMAX: %f YMIN: %f YMAX: %f", target_zone.x_min, target_zone.x_max, target_zone.y_min, target_zone.y_max);
    #endif
        // coord_t coords[] = { {125.0, 165.0, 0.0, 0.0}, { 500.0, 165.0, 0.0, 50.0 }, { 900.0, 600.0, 0.0, 50.0 }, { 1200.0, 570.0, 0.0, 50.0 } };    //125.0, 395.0, 1000, 600
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
        sima.target_zone = set_target_zone((int)rpi_socket.rx_buff[1]);
        //target_zone = set_target_zone((int)rpi_socket.rx_buff[1]);   //NE ZABORAVI NA OVO
        for (int i=0; i<data_size; i++)
        {
            memcpy(&coords[i], &rpi_socket.rx_buff[i*16+2], sizeof(coord_t));
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

        xTaskCreatePinnedToCore(app_main_2, "app_main_2", 3072, NULL, 5, NULL, 1);
    #else
        sima.tof_initialized = true;
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
	motor_init(&sima.right_motor, &motor_config, UNIT_0, TIMER_0, MR_A, MR_B);
	motor_init(&sima.left_motor, &motor_config, UNIT_0, TIMER_1, ML_A, ML_B);

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
    gptimer_alarm_init(gptimer, &gptimer_alarm, 10000, 0, 1);
    gptimer_cbs_init(gptimer, &cbs, my_callback, queue);    //Note to yourself: QueueHandle_t is a pointer of type void *. If you pass &queue, you are passing an address of the QueueHandle_t queue instead of the queue itself. ISR runs into a problem because it expects a void * pointer (which user_data parameter is)
                                                                    //You can either pass queue or (void *)queue
    // if (robot_states == ROBOT_MOVE)  gptimer_enable_and_start(gptimer);
    //-----------------------------------------------------------------------------------------------------------------------------------

    odom_init(WHEEL_CIRC, WHEEL_DIFF, CALIBRATION, &robot);
    set_start_position(coords[0].x, coords[0].y, coords[0].teta, &robot);

    esp_log_level_set(TAG_Control, ESP_LOG_INFO);
    esp_log_level_set(TAG_Control, ESP_LOG_WARN);
    esp_log_level_set("Main", ESP_LOG_INFO);
    esp_log_level_set("Spline", ESP_LOG_INFO);
    esp_log_level_set("TOF", ESP_LOG_ERROR);
    esp_log_level_set("Tof", ESP_LOG_ERROR);
    esp_log_level_set("Status", ESP_LOG_ERROR);
    esp_log_level_set("Main2", ESP_LOG_INFO);
    esp_log_level_set("Interrupt zone", ESP_LOG_NONE);
    esp_log_level_set("*", ESP_LOG_NONE);

    spline_init();

    uint8_t tof_counter = 0;

    // float x_target[3];
    // float y_target[3];
    // float theta_target[3];
    // uint8_t num_of_bezier;

    for (int j=1;j<data_size;j++)
    {
        spline_targets.x_target[j-1] = coords[j].x;
        spline_targets.y_target[j-1] = coords[j].y;
        spline_targets.theta_target[j-1] = coords[j].teta;
    }
    spline_targets.num_of_bezier = data_size - 1;
    
    while (1)
    {
        switch (sima.robot_states)
        {
            case ROBOT_WAIT_FOR_SIGNAL:
                #ifdef WIFI_USED
                    socket_recv(&rpi_socket);
                    if ((int)rpi_socket.rx_buff[0] == 1 && sima.tof_initialized)
                    { 
                        sima.robot_states = ROBOT_MOVING;
                        gptimer_enable_and_start(gptimer);
                        esp_wifi_deinit();
                        vTaskDelay(2000/portTICK_PERIOD_MS);
                    }
                #else
                    if (sima.tof_initialized) 
                    {
                        //sima.robot_states = ROBOT_MOVING;
                        sima.robot_states = ROBOT_SETUP_TARGET;
                        //vTaskDelay(5000/portTICK_PERIOD_MS);
                        gptimer_enable_and_start(gptimer);
                        vTaskDelay(2000/portTICK_PERIOD_MS);
                    }
                #endif
            break;
            case ROBOT_SETUP_TARGET:
                //setup_spline_targets(x_target, y_target, theta_target, &num_of_bezier, robot, coords);
                //sima.avoiding = true;
                /*restore_position(&robot, &temp_robot);
                encoder_reset(&encoder);
                pcnt_enable_clear_start(pcnt1);
                pcnt_enable_clear_start(pcnt2);*/
                spline_move(spline_targets.x_target, spline_targets.y_target, spline_targets.theta_target, spline_targets.num_of_bezier, 400.0, 'f', robot);
                sima.robot_states = ROBOT_MOVING;
            break;
            case ROBOT_MOVING:
                pcnt_unit_get_count(pcnt1, &encoder.right);
	            pcnt_unit_get_count(pcnt2, &encoder.left);
                calc_pos(&encoder, &robot);
                spline_compute(robot, sima.right_motor, sima.left_motor, zone);
                if (spline.stopped) sima.robot_states = ROBOT_TEMP;
                #ifdef DETECTION
                    if (check_tof(data, zone, rpi_socket) && !spline.no_detect)
                    {
                        tof_counter++;
                    }
                    if (tof_counter >= 10)
                    {
                        tof_counter = 0;
                        motors_stop(sima.right_motor, sima.left_motor);
                        /*store_position(&robot, &temp_robot);
                        disable_all_pcnts(pcnt1, pcnt2);
                        encoder_reset(&encoder);*/
                        sima.stop_time = sima.timer_count;
                        sima.robot_states = ROBOT_STOPPED;
                    }
                #endif
            break;
            case ROBOT_CHECK:
                i++;
                if (i<data_size) sima.robot_states = ROBOT_MOVING;
                else 
                {
                    sima.in_position = true;
                    sima.robot_states = ROBOT_TEMP;
                }
            break;
            case ROBOT_SERVO:
                /*servo_wave(servo1, servo2, 70.0, 20.0, 110.0, 160.0);
                vTaskDelay(10/portTICK_PERIOD_MS);*/
                ESP_LOGE("Main", "CNT: %lu", counter);
                vTaskDelay(5000000/portTICK_PERIOD_MS);
            break;
            case ROBOT_STOPPED:
                #ifdef DETECTION
                    if (!check_tof(data, zone, rpi_socket) && !sima.in_position)
                    {
                        tof_counter++;
                    }
                    if (tof_counter >= 10)
                    {
                        tof_counter = 0;
                        /*restore_position(&robot, &temp_robot);
                        encoder_reset(&encoder);
                        pcnt_enable_clear_start(pcnt1);
                        pcnt_enable_clear_start(pcnt2);*/
                        sima.robot_states = ROBOT_MOVING;
                    }
                #endif
                if (check_zone(robot, sima.target_zone)) sima.robot_states = ROBOT_TEMP;
                #ifdef AVOIDANCE
                    if (sima.timer_count > sima.stop_time + 1500)
                    {
                        ESP_LOGI("Main", "Proslo 1.5s");
                        setup_new_spline_targets(spline_targets.x_target, spline_targets.y_target, spline_targets.theta_target, &spline_targets.num_of_bezier, robot, coords);
                        sima.robot_states = ROBOT_SETUP_TARGET;
                    }
                #endif
            break;
            case ROBOT_AVOID:
                //if (check_zone(robot, sima.target_zone)) sima.robot_states = ROBOT_TEMP;
                spline_targets.x_target[0] = robot.x_pos;
                spline_targets.x_target[1] = coords[data_size - 1].x;
                spline_targets.y_target[0] = robot.y_pos - 300;
                spline_targets.y_target[1] = coords[data_size - 1].y;
                spline_targets.theta_target[0] = -1.57;
                spline_targets.theta_target[1] = coords[data_size-1].teta;
                sima.robot_states = ROBOT_SETUP_TARGET;
                /*#ifdef DETECTION
                    if (check_tof(data, zone, rpi_socket))
                    {
                        tof_counter++;
                    }
                    if (tof_counter >= 10)
                    {
                        tof_counter = 0;
                        motors_stop(sima.right_motor, sima.left_motor);
                        store_position(&robot, &temp_robot);
                        disable_all_pcnts(pcnt1, pcnt2);
                        encoder_reset(&encoder);
                        sima.stop_time = sima.timer_count;
                        sima.robot_states = ROBOT_STOPPED;
                    }
                #endif*/
                /*if (spline.stopped)
                {
                    i = data_size - 1;
                    sima.robot_states = ROBOT_MOVING;
                }*/
            break;
            case ROBOT_TEMP:
                ESP_LOGI("Main", "U robot tempu");
                vTaskDelay(10/portTICK_PERIOD_MS);
            break;
        }
        /*#ifdef DETECTION
            //ESP_LOGI("Main", "L: %d C: %d D: %d", zone.interrupt_left, zone.interrupt_center, zone.interrupt_right);
            if (check_tof(data, zone, rpi_socket) && sima.robot_states == ROBOT_MOVING)
            {
                store_position(&robot, &temp_robot);
                disable_all_pcnts(pcnt1, pcnt2);
                encoder_reset(&encoder);
                sima.stop_time = sima.timer_count;
                sima.robot_states = ROBOT_STOPPED;
            }
            if (!check_tof(data, zone, rpi_socket) && sima.robot_states == ROBOT_STOPPED && !sima.in_position)
            {
                restore_position(&robot, &temp_robot);
                encoder_reset(&encoder);
                pcnt_enable_clear_start(pcnt1);
                pcnt_enable_clear_start(pcnt2);
                sima.robot_states = ROBOT_MOVING;
            }
        #endif*/
        //ESP_LOGI("Main", "ZL: %d ZC: %d ZR: %d", zone.interrupt_left, zone.interrupt_center, zone.interrupt_right);
        
        vTaskDelay(10/portTICK_PERIOD_MS);
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
    vTaskDelay(pdMS_TO_TICKS(1000));
    sima.tof_initialized = true;

    while(1){
        if(get_tof_intr(&tof)){
            status = VL53LMZ_STATUS_OK;

            status |= VL53LMZ_Get_Distance(&tof.conf, &data);
            //status |= ConvertDist2Point(&data, &tof, 450.0);

            //tof_calculate_distances_interrupt(&tof, &data);

            // for(uint16_t i = 0; i < VL53LMZ_RESOLUTION_4X4 / 4; i++){
            //     ESP_LOGI("Distances:", "%lu %lu %lu %lu (%lu %lu %lu %lu)", 
            //                             data.ZoneResult[i * 4].Distance, data.ZoneResult[i * 4 +1].Distance, 
            //                             data.ZoneResult[i * 4+2].Distance, data.ZoneResult[i*4+3].Distance, 
            //                             data.ZoneResult[i * 4].Status, data.ZoneResult[i * 4 +1].Status, 
            //                             data.ZoneResult[i * 4+2].Status, data.ZoneResult[i*4+3].Status);
            // }        
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
}

void setup_new_spline_targets(float *x, float *y, float *theta, uint8_t *bezier_num, position_t robot, coord_t *coordinates)
{
            x[0] = robot.x_pos;
            x[1] = coordinates[data_size - 1].x;
            y[0] = robot.y_pos - 300;
            y[1] = coordinates[data_size - 1].y;
            theta[0] = -1.57;
            theta[1] = coordinates[data_size - 1].teta;
            if (theta[0] > M_PI) theta[0] -= 2*M_PI;
            if (theta[0] < -M_PI) theta[0] += 2*M_PI;
            theta[1] = 0.0 /*atan2((y_target[1]-y_target[0]), (x_target[1]-x_target[0]))*/;
            if (theta[1] > M_PI) theta[1] -= 2*M_PI;
            if (theta[1] < -M_PI) theta[1] += 2*M_PI;
            //ESP_LOGI("Main", "T: %f AT: %f", theta[0], atan2((y[1]-y[0]), (x[1]-x[0])));
            *bezier_num = 2;
}
