#include "utils.h"

void bytes_to_float(uint8_t* start, float* dest){
    memcpy(start, dest, sizeof(float));
}