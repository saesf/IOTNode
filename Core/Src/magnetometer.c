#include "magnetometer.h"
#include "stm32l4xx_hal.h"

/**
  * @defgroup    LIS3MDL_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t lis3mdl_from_fs4_to_gauss(int16_t lsb)
{
  return ((float_t)lsb / 6842.0f);
}

float_t lis3mdl_from_fs8_to_gauss(int16_t lsb)
{
  return ((float_t)lsb / 3421.0f);
}

float_t lis3mdl_from_fs12_to_gauss(int16_t lsb)
{
  return ((float_t)lsb / 2281.0f);
}

float_t lis3mdl_from_fs16_to_gauss(int16_t lsb)
{
  return ((float_t)lsb / 1711.0f);
}

float_t lis3mdl_from_lsb_to_celsius(int16_t lsb)
{
  return ((float_t)lsb / 8.0f) + (25.0f);
}

