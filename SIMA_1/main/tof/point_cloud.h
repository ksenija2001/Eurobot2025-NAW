/*
 * point_cloud.h
 *
 *  Created on: Jan 9, 2025
 *      Author: xenia
 */

 #ifndef POINT_CLOUD_H_
 #define POINT_CLOUD_H_
 
 #include "tof.h"
 #include "../utils/vector.h"
 #include "tof_api.h"
 #include <math.h>
 
 typedef struct {
     double SinPitch[64];
     double SinYaw[64];
     double CosPitch[64];
     double CosYaw[64];
 } sSinCosMap_8x8_t;
 
 int32_t ConvertDist2Point(VL53LMZ_Result_t *data, VL53LMZ_Object* dev, float max_dist);
 void Point2Bytes(sVector3_t point_cloud, uint8_t *point_bytes);
 void GenerateTables(void);
 
 
 #endif /* POINT_CLOUD_H_ */