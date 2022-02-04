/*
 * ICM20948.h
 *
 *  Created on: Jun 1, 2020
 *      Author: perSec
 */

#ifndef INC_ICM20948_H_
#define INC_ICM20948_H_


#include "I2C.h"
#include "main.h"
#include "math.h"

#define alpha 0.9998
#define dt 0.002

typedef struct __ICM20948{
	I2C_struct i2c;

	uint8_t gyro_address;
	uint8_t magneto_address;

	int32_t getaccx, getaccy, getaccz;
	float f_gyx, f_gyy, f_gyz;

	float pitch_offset, roll_offset, yaw_offset;		//pitch, roll
	int16_t gyro_offset[3];				//x,y,z

	float pitch, roll, yaw;
	int16_t temp[3];

}ICM20948;

ICM20948 icm20948;

void Gyro_Writebyte(ICM20948 * icm20948,uint8_t register_address,uint8_t data);
uint8_t Gyro_Readbyte(ICM20948 * icm20948,uint8_t register_address);

void bank_select(ICM20948* icm20948,uint8_t bank);

uint8_t ICM_who_am_i(ICM20948* icm20948);

void init_ICM20948(ICM20948* icm20948,I2C_TypeDef* I2C);

void ICM_Readaccgyro(ICM20948* icm20948);

void ICM_Gyrocali(ICM20948* icm20948);

void ICM_Angcali(ICM20948* icm20948);

void ICM_ComplementaryFilter(ICM20948* icm20948);



void Mag_Writebyte(ICM20948 * icm20948,uint8_t register_address,uint8_t data);

uint8_t Mag_Readbyte(ICM20948 * icm20948,uint8_t register_address);

uint8_t AK_Company_ID(ICM20948* icm20948);

uint8_t AK_Device_ID(ICM20948* icm20948);

void init_AK09916(ICM20948* icm20948);

void AK_ReadData(ICM20948* icm20948,float* data,int16_t* data_int);

#endif /* INC_ICM20948_H_ */
