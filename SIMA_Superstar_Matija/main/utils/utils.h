#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <string.h>
#include <math.h>

void bytes_to_float(uint8_t* start, float* dest);
void float_to_string(float num, char* dest);

#endif