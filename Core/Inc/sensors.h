#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_
#include "main.h"

#define LSDM6DSL_FIFO_THRESHOLD 732

uint8_t AccelGyroMeterInitialization();
void MagnetoMeterInitialization(lis3mdl_om_t rate, lis3mdl_fs_t scale, lis3mdl_md_t mode, uint16_t threshold);
uint8_t MagReadPulling();
uint8_t MagReadDMA();
#endif /* INC_SENSORS_H_ */
