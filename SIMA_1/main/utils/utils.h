#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <string.h>
#include <math.h>

void bytes_to_float(uint8_t* start, float* dest);
void float_to_string(float num, char* dest);
uint8_t factorial(uint8_t num);
float distance_calc(float x1, float y1, float x2, float y2);
float magnitude(float s1, float s2);
float poow(float a, uint8_t b);

#endif