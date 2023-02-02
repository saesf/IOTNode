/*
 * magnetometer.c
 *
 *  Created on: Oct 29, 2021
 *      Author: sPC
 */
#include "sensors.h"
#include "stm32l4xx_hal.h"
#include <stdlib.h>
#include "lsm6dsl_reg.h"
#include "magnetometer.h"




extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c1;
static uint8_t whoamI;


uint8_t AccelGyroMeterInitialization()
{
	/* Check device ID */
	whoamI = 0;
	HAL_I2C_Mem_Read(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &whoamI, 1, 1000);
	if ( whoamI != LSM6DSL_ID )
		return 1;

	/* Restore default configuration */
	lsm6dsl_ctrl3_c_t ctrl3_c;
	if(HAL_I2C_Mem_Read(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_CTRL3_C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl3_c, 1, 1000) == HAL_OK)
	{
		ctrl3_c.sw_reset = PROPERTY_ENABLE;
		HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_CTRL3_C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl3_c, 1, 1000);
	}


	do {
		HAL_I2C_Mem_Read(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_CTRL3_C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl3_c, 1, 1000);
	} while (ctrl3_c.sw_reset==1);

	if(HAL_I2C_Mem_Read(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_CTRL3_C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl3_c, 1, 1000) == HAL_OK)
	{
		ctrl3_c.bdu = PROPERTY_ENABLE; /* Enable Block Data Update */
		ctrl3_c.if_inc = PROPERTY_ENABLE; /* Enable register address auto-increment */
		HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_CTRL3_C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl3_c, 1, 1000);
	}


	lsm6dsl_ctrl1_xl_t ctrl1_xl;
	lsm6dsl_ctrl2_g_t ctrl2_g;
	if(HAL_I2C_Mem_Read(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl1_xl, 1, 1000) == HAL_OK)
	{
		ctrl1_xl.odr_xl = LSM6DSL_XL_ODR_6k66Hz;/* Set Output Data Rate */
		ctrl1_xl.fs_xl = LSM6DSL_2g;/* Set full scale */
		HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl1_xl, 1, 1000);
	}
	if(HAL_I2C_Mem_Read(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_CTRL2_G, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl2_g, 1, 1000) == HAL_OK)
	{
		ctrl2_g.odr_g = LSM6DSL_GY_ODR_6k66Hz;/* Set Output Data Rate */
		ctrl2_g.fs_g = LSM6DSL_2000dps;/* Set full scale */
		HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_CTRL2_G, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl2_g, 1, 1000);
	}


	/* Configure filtering chain(No aux interface) */
	/* Accelerometer - analog filter */
	if(HAL_I2C_Mem_Read(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl1_xl, 1, 1000) == HAL_OK)
	{
		ctrl1_xl.bw0_xl = LSM6DSL_XL_ANA_BW_1k5Hz;
		HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl1_xl, 1, 1000);
	}
	/* Accelerometer - LPF1 path ( LPF2 not used )*/
	//lsm6dsl_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSL_XL_LP1_ODR_DIV_4);
	/* Accelerometer - LPF1 + LPF2 path */
	lsm6dsl_ctrl8_xl_t ctrl8_xl;

	if(HAL_I2C_Mem_Read(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_CTRL8_XL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl8_xl, 1, 1000) == HAL_OK)
	{
		ctrl8_xl.input_composite = ((uint8_t) LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100 & 0x10U) >> 4;
		ctrl8_xl.hpcf_xl = (uint8_t) LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100 & 0x03U;
		ctrl8_xl.lpf2_xl_en = 1;
		ctrl8_xl.hp_slope_xl_en = 0;
		HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_CTRL8_XL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl8_xl, 1, 1000);
	}

	/* Accelerometer - High Pass / Slope path */
	//lsm6dsl_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
	//lsm6dsl_xl_hp_bandwidth_set(&dev_ctx, LSM6DSL_XL_HP_ODR_DIV_100);
	/* Gyroscope - filtering chain */
	lsm6dsl_ctrl4_c_t ctrl4_c;
	lsm6dsl_ctrl6_c_t ctrl6_c;
	lsm6dsl_ctrl7_g_t ctrl7_g;
	if(HAL_I2C_Mem_Read(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_CTRL7_G, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl7_g, 1, 1000) == HAL_OK)
	{
		ctrl2_g.fs_g = LSM6DSL_2000dps;
		ctrl7_g.hpm_g  = ((uint8_t)LSM6DSL_HP_1Hz04_LP1_AGGRESSIVE & 0x30U) >> 4;
		ctrl7_g.hp_en_g = ((uint8_t)LSM6DSL_HP_1Hz04_LP1_AGGRESSIVE & 0x80U) >> 7;
		if(HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_CTRL7_G, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl7_g, 1, 1000) == HAL_OK)
		{
			if(HAL_I2C_Mem_Read(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_CTRL6_C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl6_c, 1, 1000) == HAL_OK)
			{
				ctrl6_c.ftype = (uint8_t)LSM6DSL_HP_1Hz04_LP1_AGGRESSIVE & 0x03U;
				if(HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_CTRL6_C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl6_c, 1, 1000) == HAL_OK)
				{
					if(HAL_I2C_Mem_Read(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_CTRL4_C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl4_c, 1, 1000) == HAL_OK)
					{
						ctrl4_c.lpf1_sel_g = ((uint8_t)LSM6DSL_HP_1Hz04_LP1_AGGRESSIVE & 0x08U) >> 3;
						HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_CTRL4_C, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl4_c, 1, 1000);
					}
				}
			}
		}
	}

	/*FIFO settings*/

	lsm6dsl_fifo_ctrl1_t fifo_ctrl1;
	lsm6dsl_fifo_ctrl2_t fifo_ctrl2;
	lsm6dsl_fifo_ctrl3_t fifo_ctrl3;
	lsm6dsl_fifo_ctrl5_t fifo_ctrl5;

	fifo_ctrl1.fth=LSDM6DSL_FIFO_THRESHOLD&0xff; /*INT1 threshold, should be a multiple of 6*/
	HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_FIFO_CTRL1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&fifo_ctrl1, 1, 1000);
	fifo_ctrl2.fifo_temp_en=0;/*disable temprature and pedo fifo, set higher bits of threshold to 0*/
	fifo_ctrl2.fth=(LSDM6DSL_FIFO_THRESHOLD>>8);
	fifo_ctrl2.timer_pedo_fifo_drdy=0;
	fifo_ctrl2.timer_pedo_fifo_en=0;
	HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_FIFO_CTRL2, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&fifo_ctrl2, 1, 1000);
	fifo_ctrl3.dec_fifo_gyro=LSM6DSL_FIFO_DS3_NO_DEC; /*no decimation for gyro data*/
	fifo_ctrl3.dec_fifo_xl=LSM6DSL_FIFO_DS3_NO_DEC; /*no decimation for accel data*/
	HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_FIFO_CTRL3, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&fifo_ctrl3, 1, 1000);
	fifo_ctrl5.odr_fifo=LSM6DSL_FIFO_1k66Hz;
	fifo_ctrl5.fifo_mode=LSM6DSL_STREAM_MODE;
	HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_FIFO_CTRL5, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&fifo_ctrl5, 1, 1000);


	lsm6dsl_int1_ctrl_t int1_ctrl;
	if(HAL_I2C_Mem_Read(&SENSOR_BUS, LSM6DSL_I2C_ADD_R, LSM6DSL_INT1_CTRL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&int1_ctrl, 1, 1000) == HAL_OK)
	{
		int1_ctrl.int1_fth=PROPERTY_ENABLE;
		HAL_I2C_Mem_Write(&SENSOR_BUS, LSM6DSL_I2C_ADD_W, LSM6DSL_INT1_CTRL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&int1_ctrl, 1, 1000);
	}

	return 0;
}

void MagnetoMeterInitialization(lis3mdl_om_t rate, lis3mdl_fs_t scale, lis3mdl_md_t mode, uint16_t threshold)
{
	uint8_t whoami,rst;
	lis3mdl_reg_t regs;

	//Device Who am I[get]
	regs.byte=0;
	if (HAL_I2C_Mem_Read(&hi2c2,LIS3MDL_I2C_ADD_R ,LIS3MDL_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &regs,1,1) != HAL_OK)
	{
		Error_Handler();
	}
	if (regs.byte != LIS3MDL_ID)
		Error_Handler(); /*manage here device not found */


	//Software reset. Restore the default values in user registers[set]
	regs.byte=0;
	if (HAL_I2C_Mem_Read(&hi2c2,LIS3MDL_I2C_ADD_R ,LIS3MDL_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regs,1,1) != HAL_OK)
	{
		Error_Handler();
	}
	regs.ctrl_reg2.soft_rst = PROPERTY_ENABLE;
	if (HAL_I2C_Mem_Write(&hi2c2,LIS3MDL_I2C_ADD_W ,LIS3MDL_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regs,1,1) != HAL_OK)
	{
		Error_Handler();
	}

	//wait for soft-reset to take effect
	do {
		if (HAL_I2C_Mem_Read(&hi2c2,LIS3MDL_I2C_ADD_R ,LIS3MDL_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regs,1,1) != HAL_OK)
		{
			Error_Handler();
		}
	} while (regs.ctrl_reg2.soft_rst==1);

	//set scale
	regs.ctrl_reg2.fs = scale;
	if (HAL_I2C_Mem_Write(&hi2c2,LIS3MDL_I2C_ADD_W ,LIS3MDL_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regs,1,1) != HAL_OK)
	{
		Error_Handler();
	}

	//temperature enable, set conversion rate
	regs.byte=0;
	regs.ctrl_reg1.temp_en = PROPERTY_ENABLE;
	regs.ctrl_reg1.om = rate;
	regs.ctrl_reg1.st = PROPERTY_DISABLE;
	if (HAL_I2C_Mem_Write(&hi2c2,LIS3MDL_I2C_ADD_W ,LIS3MDL_CTRL_REG1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regs,1,1) != HAL_OK)
	{
		Error_Handler();
	}

	//conversion mode
	regs.byte=0;
	regs.ctrl_reg3.md = mode;
	if (HAL_I2C_Mem_Write(&hi2c2,LIS3MDL_I2C_ADD_W ,LIS3MDL_CTRL_REG3, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regs,1,1) != HAL_OK)
	{
		Error_Handler();
	}

	//set conversion rate on Z axis
	regs.byte=0;
	regs.ctrl_reg4.omz = (uint8_t)(((uint8_t) rate >> 4) & 0x03U);
	if (HAL_I2C_Mem_Write(&hi2c2,LIS3MDL_I2C_ADD_W ,LIS3MDL_CTRL_REG4, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regs,1,1) != HAL_OK)
	{
		Error_Handler();
	}


	/* Enable Block Data Update/ disable fast-read */
	regs.byte=0;
	regs.ctrl_reg5.bdu = PROPERTY_ENABLE;
	regs.ctrl_reg5.fast_read = PROPERTY_DISABLE;
	if (HAL_I2C_Mem_Write(&hi2c2,LIS3MDL_I2C_ADD_W ,LIS3MDL_CTRL_REG5, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regs,1,1) != HAL_OK)
	{
		Error_Handler();
	}

//	//set interrupt threshold
//	uint8_t buff[2];
//	buff[1] = (uint8_t)(threshold / 256U);
//	buff[0] = (uint8_t)(threshold - (buff[1] * 256U));
//	if (HAL_I2C_Mem_Write(&hi2c2,LIS3MDL_I2C_ADD_W ,LIS3MDL_INT_THS_L, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&buf,2,1) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	//enable interrupt
//	regs.byte=0;
//	if (HAL_I2C_Mem_Read(&hi2c2,LIS3MDL_I2C_ADD_R ,LIS3MDL_INT_CFG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regs,1,1) != HAL_OK)
//	{
//		Error_Handler();
//	}
//	regs.int_cfg.iea = PROPERTY_ENABLE;
//	regs.int_cfg.ien = PROPERTY_ENABLE;
//	regs.int_cfg.zien = PROPERTY_ENABLE;
//	regs.int_cfg.yien = PROPERTY_ENABLE;
//	regs.int_cfg.xien = PROPERTY_ENABLE;
//	regs.int_cfg.lir = 1;
//	if (HAL_I2C_Mem_Write(&hi2c2,LIS3MDL_I2C_ADD_H ,LIS3MDL_INT_CFG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regs,1,1) != HAL_OK)
//	{
//		Error_Handler();
//	}
}

uint8_t magFlag=1;
uint8_t magDMA=1;
uint8_t lsdm6dslDmaCpltFlg=1;

HAL_StatusTypeDef res;
uint32_t errorCounter=0;
uint8_t MagReadDMA()
{
//	memset(magnetometerPayload,0,8);
//	HAL_StatusTypeDef res= HAL_I2C_Mem_Read_DMA(&hi2c2,LIS3MDL_I2C_ADD_R ,LIS3MDL_OUT_X_L|0x80, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&magnetometerPayload,8);
	HAL_StatusTypeDef res= HAL_I2C_Mem_Read_DMA(&hi2c2,LIS3MDL_I2C_ADD_R ,LIS3MDL_OUT_X_L, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&magnetometerPayload,10);
	if( res!= HAL_OK)
	{
//		Error_Handler();
		printf("res=%d\n\r",res);
		errorCounter++;
		return 1;
	}
	return 0;
}

uint8_t MagReadPulling()
{
	res = HAL_I2C_Mem_Read(&hi2c2,LIS3MDL_I2C_ADD_R ,LIS3MDL_OUT_X_L, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&magnetometerPayload,8, 1000);
	if( res!= HAL_OK)
	{
		Error_Handler();
	}
	return 0;
}

void I2CMultiByteReadDMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
        uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
//	res = HAL_I2C_Mem_Read(&hi2c2,LIS3MDL_I2C_ADD_R ,LIS3MDL_OUT_X_L, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&magnetometerPayload,8, 1000);
}
