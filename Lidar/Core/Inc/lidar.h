
#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stm32g4xx_hal.h"
#include "dma.h"
#include "vector.h"
#include "point_cloud.h"
//#include "svd.h"
#include "fdcan.h"


#define BUFFER_SIZE 132
#define PWM_ARR 5759
#define PWM_SENS 0.0735294117647
#define PWM_SENS_OFFSET 6.176470588235

// Request packet
#define START          0xA5
#define STOP           0x25
#define RESET          0x40
#define SCAN           0x20
#define EXPRESS_SCAN   0x82
#define FORCE_SCAN     0x21
#define GET_INFO       0x50
#define GET_HEALTH     0x52
#define GET_SAMPLERATE 0x59
#define GET_LIDAR_CONF 0x84
#define MOTOR_SPEED    0xA8

// Response packet
#define START2         0x5A

// Opponent robot
#define BEACON_SUPPORT_DIAMETER 70  // mm
#define LIDAR_FOV 120
#define LIDAR_SIDE_DISTANCE 400

// Ultra Cabin parsing
#define MAX_ULTRA_CABINS 32
#define RPLIDAR_VARBITSCALE_X2_SRC_BIT 9
#define RPLIDAR_VARBITSCALE_X4_SRC_BIT 11
#define RPLIDAR_VARBITSCALE_X8_SRC_BIT 12
#define RPLIDAR_VARBITSCALE_X16_SRC_BIT 14

#define RPLIDAR_VARBITSCALE_X2_DEST_VAL 512
#define RPLIDAR_VARBITSCALE_X4_DEST_VAL 1280
#define RPLIDAR_VARBITSCALE_X8_DEST_VAL 1792
#define RPLIDAR_VARBITSCALE_X16_DEST_VAL 3328

// Triangulation
#define MIN_POINTS 10
#define 	adjust_value_to_bounds(value, max)   ( ( value > max ) ? max : ( ( value < -max ) ? -max : value ) )
#define 	cot(x)   ( 1 / tan(x) )
#define 	Cot(x)   cot(x)
#define 	COT_MAX   100000000

typedef struct {
    int32_t major;
    int32_t predict1;
    int32_t predict2;
} RPlidarUltraCabin;

typedef struct {
    uint8_t sync_bit;
    uint16_t angle_q6;
    uint32_t dist_q2;
} RPlidarMeasurementHQ;

typedef struct {
	float x;
	float y;
	float theta;
	float speed;
	float ang_speed;
} sOdom_t;

// Response descriptor struct
typedef struct {
	uint32_t length;
	uint8_t send_mode; // 0 - signel response, 1 - multiple response, 4 - all responses received
	uint8_t data_type;
	uint8_t packet_num;
} sDescriptor_t;

// Lidar hardware information
typedef struct{
	uint8_t model;
	uint8_t firmware_minor;
	uint8_t firmware_major;
	uint8_t hardware;
	uint16_t standard_samplerate;
	uint16_t express_samplerate;
} sInfo_t;

// Lidar scan mode informations
typedef struct{
	uint32_t sample_duration;
	uint32_t max_distance;
	uint8_t  answer_type;
	uint8_t  name[20];
} sScanMode_t;

// Lidar health data
typedef struct{
	uint8_t status;
	uint16_t error_code;
} sHealth_t;

typedef struct{
	uint8_t sync;          // identifies the start of a new response packet - 0xA5
	uint8_t checksum;      // XOR of all data bytes in response packet
    uint16_t start_angle_q6;  // reference value for the angle data in current response packet
    float start_angle;
	uint8_t S;             // start flag of new scan
	RPlidarUltraCabin ultra_cabins[MAX_ULTRA_CABINS];
} sResponse_t;

typedef struct{
	float distance1;
	float distance2;
	float theta1;
	float theta2;
} sCabin_t;

typedef struct{
	uint16_t ccr1;
	int8_t inc;
} sPWM_t;

typedef struct{
	uint16_t front;
	uint16_t back;
} sDetection_t;

// Start/Stop lidar
void Lidar_Start(TIM_HandleTypeDef* motor_htim, TIM_HandleTypeDef* ramp_htim, TIM_HandleTypeDef* parse_htim, UART_HandleTypeDef* huart);
void Lidar_Stop_All(TIM_HandleTypeDef* motor_htim, TIM_HandleTypeDef* ramp_htim, TIM_HandleTypeDef* parse_htim,  UART_HandleTypeDef* huart);

// Commands without response
void Lidar_Stop(UART_HandleTypeDef *huart);
void Lidar_Reset(UART_HandleTypeDef *huart);
void Lidar_Unknown(UART_HandleTypeDef *huart);

// Commands with response
void Lidar_Get_Health(UART_HandleTypeDef *huart);
void Lidar_Get_Samplerate(UART_HandleTypeDef *huart);
void Lidar_Get_Lidar_Conf(UART_HandleTypeDef *huart, uint8_t config, uint8_t request_length, uint8_t mode);
void Lidar_Get_Info(UART_HandleTypeDef *huart);
void Lidar_Scan(UART_HandleTypeDef *huart);
void Lidar_Express_Scan(UART_HandleTypeDef *huart, uint8_t scan_mode_id);

// Helper functions
void Lidar_Motor_Stop(TIM_HandleTypeDef *tim, uint8_t channel);
void Lidar_Motor_Speed(TIM_HandleTypeDef *tim, uint8_t channel, uint16_t rpm, TIM_HandleTypeDef *tim_rpm);
uint8_t Lidar_CRC(uint8_t msg[], uint8_t length, uint8_t start);
void Process_Distance(float distance, float angle, uint8_t new_scan);
void TIM6_IT(TIM_HandleTypeDef *tim);
void TIM7_IT(TIM_HandleTypeDef *tim);

uint16_t Segment_PC(sVector3_t* pc, uint16_t ind, uint16_t radius);
void Get_Beacons();
void Get_Opponent();
uint8_t Choose_Beacon(sVector3_t* position, sVector3_t* point);
uint8_t Check_Beacon_Points();
uint8_t VarbitScale_Decode(int32_t scaled, uint32_t *decoded);

float triangulationPierlot(sVector3_t *new_robot, sVector3_t beacon1, sVector3_t beacon2, sVector3_t beacon3);

extern uint8_t rx_buff[BUFFER_SIZE];
extern sDescriptor_t response_desc;
extern sOdom_t self;
extern sDetection_t detection;
extern uint8_t process_beacon;
extern uint8_t process_opponent;
extern uint8_t color;
extern float speed;
extern float ang_speed;


#endif /* INC_LIDAR_H_ */
