/*
 * utils.c
 *
 *  Created on: Jan 31, 2025
 *      Author: xenia
 */

#include "utils.h"

union U_F
{
	float f;
	uint8_t u[4];
} convert_float;

union U_I
{
	int32_t i;
	uint8_t u[2];
} convert_int;

// Converts a float number into an array of 3 bytes
void Float2Bytes(uint8_t *buffer, uint8_t start, float data)
{
	convert_float.f = data;

	for (uint8_t i = 0; i < 4; ++i)
	{
		buffer[start + i] = convert_float.u[i];
	}
}

// Converts uint8_t bytes into a float number
float Bytes2Float(uint8_t msg[], uint8_t start)
{
	convert_float.u[0] = msg[start];
	convert_float.u[1] = msg[start + 1];
	convert_float.u[2] = msg[start + 2];
	convert_float.u[3] = msg[start + 3];

	return convert_float.f;
}

// Converts uint8_t bytes into a int16 number
int16_t Bytes2Int32(uint8_t msg[], uint8_t start)
{
	convert_int.u[0] = msg[start];
	convert_int.u[1] = msg[start + 1];
	convert_int.u[2] = msg[start + 2];
	convert_int.u[3] = msg[start + 3];

	return convert_int.i;
}

void Int162Bytes(uint8_t *buffer, uint8_t start, int16_t data)
{
	convert_int.i = data;

	buffer[start] = convert_int.u[0];
	buffer[start + 1] = convert_int.u[1];
}

uint16_t factorial(uint8_t num){
	uint16_t res = 1;
	for (uint8_t i=1; i<=num;++i){
		res *= i;
	}
	return res;
}

float poow(float a, int exp){
	float res = 1;
	for(int i=0; i<exp; i++){
		res *= a;
	}
	return res;;
}

float distance(float x0, float y0, float x1, float y1){
	return sqrtf(poow(x1-x0, 2) + poow(y1-y0, 2));
}

float magnitude(float x, float y){
	return sqrtf(x*x + y*y);
}

float normalize(float theta){
	if(theta > M_PI){
		theta -= 2*M_PI;
	}
	if(theta < -M_PI){
		theta += 2*M_PI;
	}
	return theta;
 }


