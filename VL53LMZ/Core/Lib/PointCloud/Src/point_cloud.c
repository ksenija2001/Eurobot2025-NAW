/*
 * point_cloud.c
 *
 *  Created on: Jan 9, 2025
 *      Author: xenia
 */

#include "point_cloud.h"

extern UART_HandleTypeDef huart2;

const double VL53L7CX_Pitch_8x8[64] = {
		55.11, 59.09, 62.00, 63.55, 63.55, 62.00, 59.09, 55.11,

		59.09, 64.36, 68.52, 70.93, 70.93, 68.52, 64.36, 59.09,

		62.00, 68.52, 74.29, 78.17, 78.17, 74.29, 68.52, 62.00,

		63.55, 70.93, 78.17, 84.70, 84.70, 78.17, 70.93, 63.55,

		63.55, 70.93, 78.17, 84.70, 84.70, 78.17, 70.93, 63.55,

		62.00, 68.52, 74.29, 78.17, 78.17, 74.29, 68.52, 62.00,

		59.09, 64.36, 68.52, 70.93, 70.93, 68.52, 64.36, 59.09,

		55.11, 59.09, 62.00, 63.55, 63.55, 62.00, 59.09, 55.11
};

const double VL53L5CX_Pitch_8x8[64] = {
		63.16, 66.41, 68.83, 70.14, 70.14, 68.83, 66.41, 63.16,

		66.41, 70.49, 73.77, 75.68, 75.68, 73.77, 70.49, 66.41,

		68.83, 73.77, 78.15, 81.12, 81.12, 78.15, 73.77, 68.83,

		70.14, 75.68, 81.12, 86.03, 86.03, 81.12, 75.68, 70.14,

		70.14, 75.68, 81.12, 86.03, 86.03, 81.12, 75.68, 70.14,

		68.83, 73.77, 78.15, 81.12, 81.12, 78.15, 73.77, 68.83,

		66.41, 70.49, 73.77, 75.68, 75.68, 73.77, 70.49, 66.41,

		63.16, 66.41, 68.83, 70.14, 70.14, 68.83, 66.41, 63.16
};

const double VL53LX_Yaw_8x8[64] = {
		-135.00, -125.54, -113.20,  -98.13,  -81.87,  -66.80,  -54.46,  -45.00,

		-144.46, -135.00, -120.96, -101.31,  -78.69,  -59.04,  -45.00,  -35.54,

		-156.80, -149.04, -135.00, -108.43,  -71.57,  -45.00,  -30.96,  -23.20,

		-171.87, -168.69, -161.57, -135.00,  -45.00,  -18.43,  -11.31,   -8.13,

		 171.87,  168.69,  161.57,  135.00,   45.00,   18.43,   11.31,    8.13,

		 156.80,  149.04,  135.00,  108.43,   71.57,   45.00,   30.96,   23.20,

		 144.46,  135.00,  120.96,  101.31,   78.69,   59.04,   45.00,   35.54,

		 135.00,  125.54,  113.20,   98.13,   81.87,   66.80,   54.46,   45.00
};

sSinCosMap_8x8_t map[3];

union U_F{
	float f;
	uint8_t u[4];
} convert_x, convert_y, convert_z;

uint8_t point_bytes[13];
float hypothenus;
float distance;
float roll, pitch, yaw;
float x, y, z;

double deg2rad(double deg){
	return deg * (M_PI/180.0);
}

void GenerateTables(){
	sSinCosMap_8x8_t VL53L7CX_map;
	sSinCosMap_8x8_t VL53L5CX_map;

	for (uint8_t i=0; i<64; ++i){
		VL53L7CX_map.SinPitch[i] = sin(deg2rad(VL53L7CX_Pitch_8x8[i]));
		VL53L7CX_map.CosPitch[i] = cos(deg2rad(VL53L7CX_Pitch_8x8[i]));
		VL53L7CX_map.CosYaw[i] = cos(deg2rad(VL53LX_Yaw_8x8[i]));
		VL53L7CX_map.SinYaw[i] = sin(deg2rad(VL53LX_Yaw_8x8[i]));

		VL53L5CX_map.SinPitch[i] = sin(deg2rad(VL53L5CX_Pitch_8x8[i]));
		VL53L5CX_map.CosPitch[i] = cos(deg2rad(VL53L5CX_Pitch_8x8[i]));
		VL53L5CX_map.CosYaw[i] = cos(deg2rad(VL53LX_Yaw_8x8[i]));
		VL53L5CX_map.SinYaw[i] = sin(deg2rad(VL53LX_Yaw_8x8[i]));
	}

	map[VL53LMZ_MODULE_TYPE_L5] = VL53L5CX_map;
	map[VL53LMZ_MODULE_TYPE_L7] = VL53L7CX_map;
}

int32_t ConvertDist2Point(VL53LMZ_Result_t *data, VL53LMZ_Object* dev, float max_dist){
	uint8_t model = dev->conf.module_type;
	for(uint8_t zone=0; zone < data->NumberOfZones; ++zone){
		if( (data->ZoneResult[zone].Distance > 0 && data->ZoneResult[zone].Distance < max_dist) &&
			data->ZoneResult[zone].Status == 0 ){

			distance = (float)data->ZoneResult[zone].Distance;
			dev->point_cloud[zone].vector[2] = distance;

			hypothenus = distance / map[model].SinPitch[zone];
			dev->point_cloud[zone].vector[0] = map[model].CosYaw[zone] * map[model].CosPitch[zone] * hypothenus;
			dev->point_cloud[zone].vector[1] = map[model].SinYaw[zone] * map[model].CosPitch[zone] * hypothenus;

			roll = dev->orient_offset.vector[0];
			pitch = dev->orient_offset.vector[1];
			yaw = dev->orient_offset.vector[2];
			// rotate about origin if necessary
			if (roll != 0 || pitch != 0 || yaw != 0){
				x = dev->point_cloud[zone].vector[0];
				y = dev->point_cloud[zone].vector[1];
				z = dev->point_cloud[zone].vector[2];

				dev->point_cloud[zone].vector[0] =  x*cos(pitch)*cos(yaw) + y*(sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw)) + z*(cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw));
				dev->point_cloud[zone].vector[1] =  x*cos(pitch)*sin(yaw) + y*(sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw)) + z*(cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw));
				dev->point_cloud[zone].vector[2] = -x*sin(pitch)          + y*sin(roll)*cos(pitch)                                 + z*cos(roll)*cos(pitch);
			}

			// translate to origin (center TOF position)
			dev->point_cloud[zone].vector[0] += dev->trans_offset.vector[0];
			dev->point_cloud[zone].vector[1] += dev->trans_offset.vector[1];
			dev->point_cloud[zone].vector[2] += dev->trans_offset.vector[2];

			Point2Bytes(dev->point_cloud[zone], point_bytes);
			HAL_UART_Transmit(&huart2, point_bytes, 13, 100);
		} else {
			dev->point_cloud[zone].vector[0] = 0;
			dev->point_cloud[zone].vector[1] = 0;
			dev->point_cloud[zone].vector[2] = 0;
		}
	}

	return VL53LMZ_STATUS_OK;
}

void Point2Bytes(sVector3_t point, uint8_t *point_bytes){
	convert_x.f = point.vector[0];
	convert_y.f = point.vector[1];
	convert_z.f = point.vector[2];

	point_bytes[0] = 0xa5;
	for(uint8_t j=0; j<4; ++j){
		point_bytes[j+1] = convert_x.u[j];
		point_bytes[j+4+1] = convert_y.u[j];
		point_bytes[j+8+1] = convert_z.u[j];
	}

}
