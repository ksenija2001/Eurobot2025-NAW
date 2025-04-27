#include "main.h"

Socket rpi_socket = {
    .tag = "RPI"
};

robot_states_e robot_states = ROBOT_MOVE;

int right_count = 0;
int left_count = 0;
uint64_t timer_count = 0;

encoder_info_queue_t encoder;
QueueHandle_t queue;
position_t robot;
coord_t coordinates[] = { {0.0, 0.0, 0.0}, {4000.0, 0.1, 0.0}, {1300.0, -420.0, 0.0} };    //125.0, 395.0, 1000, 600
int i = 0;
int data_size = sizeof(coordinates)/12.0;

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

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    if (i==1)
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
    //xQueueSendFromISR(queue, &encoder, &high_task_awoken);
	timer_count += 10;
    return (high_task_awoken == pdTRUE);
}

void app_main(void)
{
    float speed = 50.0;
	/*queue = xQueueCreate(100, sizeof(encoder_info_queue_t));
	if (!queue)
	{
		printf("Queue nije dobro kreiran");
	}*/

    //WIFI
    /*init_wifi(WIFI_SSID, WIFI_PASS);
    init_socket(&rpi_socket);
    socket_connect(&rpi_socket, SERVER_IP, SERVER_PORT);
    socket_transmit(&rpi_socket, SIMA_ID);
    socket_recv(&rpi_socket);   //Size of data
    ESP_LOGI("RPi", "%d", rpi_socket.rx_buff[0] + '0');
    ESP_LOGI("RPi", "Primio");
    int data_size = (int)rpi_socket.rx_buff[0]*3;
    for (int i=0; i<data_size; i++)
    {
        uint8_t raw_bytes[4] = {rpi_socket.rx_buff[i*4+1], rpi_socket.rx_buff[i*4+2], rpi_socket.rx_buff[i*4+3], rpi_socket.rx_buff[i*4+4]}; // big-endian
        float value;
        memcpy(&value, raw_bytes, sizeof(float));
        ESP_LOGW("Broj", "%f", value);
    }*/
    //-----------------------------------------------------------------------------------------------------------------------------

    //MICROSWITCH
    gpio_config_t microswitch;
    microswitch_init(&microswitch, GPIO_INTR_POSEDGE, GPIO_IN, MICROSWITCH, GPIO_PU_DIS, GPIO_PD_DIS);
    microswitch_interrupt(0, MICROSWITCH, gpio_isr_handler);
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
    gptimer_alarm_init(gptimer, &gptimer_alarm, 10000, 0, 1);
    gptimer_cbs_init(gptimer, &cbs, my_callback, queue);    //Note to yourself: QueueHandle_t is a pointer of type void *. If you pass &queue, you are passing an address of the QueueHandle_t queue instead of the queue itself. ISR runs into a problem because it expects a void * pointer (which user_data parameter is)
                                                                    //You can either pass queue or (void *)queue
    gptimer_enable_and_start(gptimer);
    //-----------------------------------------------------------------------------------------------------------------------------------

    odom_init(WHEEL_CIRC, WHEEL_DIFF, CALIBRATION, &robot);
    set_start_position(coordinates[0].x, coordinates[0].y, coordinates[0].teta, &robot);

    while (1)
    {
        switch(robot_states)
        {
            case ROBOT_WAIT_FOR_SIGNAL:
                vTaskDelay(10/portTICK_PERIOD_MS);
            break;
            case ROBOT_MOVE:
                go_to_xy(coordinates[i+1].x, coordinates[i+1].y, speed, &encoder, &robot, right_motor, left_motor, pcnt1, pcnt2);
                if (finished_movement) robot_states = ROBOT_CHECK;
            break;
            case ROBOT_CHECK:
                i++;
                if (i<data_size-1)
                { 
                    speed = 4.0;
                    robot_states = ROBOT_MOVE;
                }
                else robot_states = ROBOT_SERVO;
            break;
            case ROBOT_SERVO:
                calc_pos(&encoder, &robot);
                ESP_LOGW("stejt", "U stejtu 2 si");
                vTaskDelay(10/portTICK_PERIOD_MS);
            break;
            case ROBOT_AVOID:
                vTaskDelay(10/portTICK_PERIOD_MS);
            break;
        }
    }
}

