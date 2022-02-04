

/*
 * ICM20948.c
 *
 *  Created on: Jun 1, 2020
 *      Author: perSec
*/



#include "ICM20948_reg.h"
#include "ICM20948.h"



//========================================================================================= GYRO

static uint8_t BANK=0;

void Gyro_Writebyte(ICM20948 * icm20948,uint8_t register_address,uint8_t data){
	uint8_t Trans[2]={register_address, data};
	I2C_Transmit(&icm20948->i2c,icm20948->gyro_address,Trans,2);

}
uint8_t Gyro_Readbyte(ICM20948 * icm20948,uint8_t register_address){
	uint8_t Trans[1]={register_address};
	uint8_t Receive[1];
	I2C_Transmit(&icm20948->i2c,icm20948->gyro_address,Trans,1);
	I2C_Receive(&icm20948->i2c,icm20948->gyro_address,Receive,1);
	return Receive[0];
}

void bank_select(ICM20948* icm20948,uint8_t bank){
	if(BANK!=bank && bank<4){
		Gyro_Writebyte(icm20948,REG_BANK_SEL,bank<<4);
		BANK=bank;
	}
}

uint8_t ICM_who_am_i(ICM20948* icm20948){
	if(BANK!=0X00){
		bank_select(icm20948,0x00);
	}
	return Gyro_Readbyte(icm20948,WHO_AM_I_ICM20948);
}

void init_ICM20948(ICM20948* icm20948,I2C_TypeDef* I2C){
	icm20948->i2c.I2C=I2C;
	icm20948->gyro_address=ICM20948_ADDRESS<<1;
	icm20948->magneto_address=AK09916_ADDRESS<<1;
	LL_mDelay(1000);
	bank_select(icm20948,0);
	Gyro_Writebyte(icm20948,PWR_MGMT_1,0x80);

	LL_mDelay(100);
	Gyro_Writebyte(icm20948,PWR_MGMT_1,0x09);

	bank_select(icm20948,2);
	Gyro_Writebyte(icm20948,GYRO_SMPLRT_DIV,0x01);
	Gyro_Writebyte(icm20948,GYRO_CONFIG_1,0x03);

	Gyro_Writebyte(icm20948,ACCEL_SMPLRT_DIV_2,0x01);
	Gyro_Writebyte(icm20948,ACCEL_CONFIG,0x03);
	bank_select(icm20948,0);
	Gyro_Writebyte(icm20948,INT_PIN_CFG,0x02);		//bypass enable
}

void ICM_Readaccgyro(ICM20948* icm20948){
	int16_t temp[3];
	int16_t tempaccx,tempaccy,tempaccz;
	uint8_t databuf[12];
	uint8_t reg;
	bank_select(icm20948,0);
	reg=ACCEL_XOUT_H;
	I2C_Transmit(&icm20948->i2c,icm20948->gyro_address,&reg,1);
	I2C_Receive(&icm20948->i2c,icm20948->gyro_address,databuf,12);
	tempaccx=(int16_t)(databuf[0]<<8 | databuf[1]);
	tempaccy=(int16_t)(databuf[2]<<8 | databuf[3]);
	tempaccz=(int16_t)(databuf[4]<<8 | databuf[5]);
	temp[0]=(int16_t)(databuf[6]<<8 | databuf[7]);
	temp[1]=(int16_t)(databuf[8]<<8 | databuf[9]);
	temp[2]=(int16_t)(databuf[10]<<8 | databuf[11]);
	icm20948->getaccx=tempaccx;
	icm20948->getaccy=tempaccy;
	icm20948->getaccz=tempaccz;
	icm20948->temp[0]=temp[0];
	icm20948->temp[1]=temp[1];
	icm20948->temp[2]=temp[2];
	icm20948->f_gyx=((float)(temp[0]-icm20948->gyro_offset[0]))/65.5;
	icm20948->f_gyy=((float)(temp[1]-icm20948->gyro_offset[1]))/65.5;
	icm20948->f_gyz=((float)(temp[2]-icm20948->gyro_offset[2]))/65.5;
	/*printf("%d %d %d\r\n",icm20948->getmpuaccx,icm20948->getmpuaccy,icm20948->getmpuaccz);
	printf("%.1f %.1f %.1f\r\n\n\r",icm20948->f_gyx,icm20948->f_gyy,icm20948->f_gyz);*/
}

void ICM_Gyrocali(ICM20948* icm20948){
	int32_t temp32[3]={0,0,0};
	int16_t temp16[3];
	uint8_t databuf[12];
	uint8_t reg;
	LL_mDelay(100);
	bank_select(icm20948,0);
	for(uint32_t i=0;i<1000;i++){
		if(i%100==0)
			printf("%d\r\n",i);
		reg=ACCEL_XOUT_H;
		I2C_Transmit(&icm20948->i2c,icm20948->gyro_address,&reg,1);
		I2C_Receive(&icm20948->i2c,icm20948->gyro_address,databuf,12);
		temp16[0]=(int16_t)(databuf[6]<<8 | databuf[7]);
		temp16[1]=(int16_t)(databuf[8]<<8 | databuf[9]);
		temp16[2]=(int16_t)(databuf[10]<<8 | databuf[11]);
		temp32[0]+=temp16[0];
		temp32[1]+=temp16[1];
		temp32[2]+=temp16[2];

	}
	icm20948->gyro_offset[0]=temp32[0]/1000;
	icm20948->gyro_offset[1]=temp32[1]/1000;
	icm20948->gyro_offset[2]=temp32[2]/1000;
	printf("gy %d %d %d\r\n",icm20948->gyro_offset[0],icm20948->gyro_offset[1],icm20948->gyro_offset[2]);
}

void ICM_Angcali(ICM20948* icm20948){
	uint8_t databuf[12];
	int16_t temp[6];
	int32_t gettemp[6];
	uint8_t reg;
	float temp_offset[3];
	float acctotvec,accdegx,accdegy,accdegz;
	bank_select(icm20948,0);
	for(uint32_t i=0;i<1000;i++){
		if(i%100==0)
			printf("%d\r\n",i);
		reg=ACCEL_XOUT_H;
		I2C_Transmit(&icm20948->i2c,icm20948->gyro_address,&reg,1);
		I2C_Receive(&icm20948->i2c,icm20948->gyro_address,databuf,12);
		temp[0]=((databuf[0]<<8)|databuf[1]);
		temp[1]=((databuf[2]<<8)|databuf[3]);
		temp[2]=((databuf[4]<<8)|databuf[5]);
		temp[3]=((databuf[6]<<8)|databuf[7]);
		temp[4]=((databuf[8]<<8)|databuf[9]);
		temp[5]=((databuf[10]<<8)|databuf[11]);
		gettemp[0]=temp[0];
		gettemp[1]=temp[1];
		gettemp[2]=temp[2];
		gettemp[3]=(temp[3]-icm20948->gyro_offset[0])/65;
		gettemp[4]=(temp[4]-icm20948->gyro_offset[0])/65;
		gettemp[5]=(temp[5]-icm20948->gyro_offset[0])/65;

		acctotvec=sqrt((float)(gettemp[0]*gettemp[0]/100+gettemp[1]*gettemp[1]/100+gettemp[2]*gettemp[2]/100))*10;
		accdegx=asin((float)gettemp[0]/acctotvec)*(57.29577951);
		accdegy=asin((float)gettemp[1]/acctotvec)*(57.29577951);
		accdegz=asin((float)gettemp[2]/acctotvec)*(57.29577951);
		temp_offset[0]+=accdegx;
		temp_offset[1]+=accdegy;
		temp_offset[2]+=accdegz;
	}
	icm20948->pitch_offset=temp_offset[0]/1000;
	icm20948->roll_offset=temp_offset[1]/1000;
	icm20948->yaw_offset=temp_offset[2]/1000;
	icm20948->pitch=icm20948->pitch_offset;
	icm20948->roll=icm20948->roll_offset;
	icm20948->yaw=icm20948->yaw_offset;
//	printf("p:%.1f r:%.1f\r\n",icm20948->pitch_offset,icm20948->roll_offset);
}

void ICM_ComplementaryFilter(ICM20948* icm20948){
	float accdegx,accdegy,accdegz,acctotvec;
	ICM_Readaccgyro(icm20948);

	acctotvec=sqrtf((float)(icm20948->getaccx*icm20948->getaccx/100
			+icm20948->getaccy*icm20948->getaccy/100
			+icm20948->getaccz*icm20948->getaccz/100))*10;
	accdegx=asinf((float)icm20948->getaccx/acctotvec)*(57.29577951);
	accdegy=asinf((float)icm20948->getaccy/acctotvec)*(57.29577951);
	accdegz=asinf((float)icm20948->getaccz/acctotvec)*(57.29577951);

	icm20948->pitch=(alpha)*(icm20948->pitch-(icm20948->f_gyy)*dt)+(1-alpha)*(accdegx);
	icm20948->roll=(alpha)*(icm20948->roll-(icm20948->f_gyx)*dt)-(1-alpha)*(accdegy);
	icm20948->yaw=(alpha)*(icm20948->yaw-(icm20948->f_gyz)*dt)-(1-alpha)*(accdegz);
	/*printf("%d %d %d\r\n",icm20948->getmpuaccx,icm20948->getmpuaccy,icm20948->getmpuaccz);
	printf("%.1f %.1f %.1f\r\n",icm20948->f_gyx,icm20948->f_gyy,icm20948->f_gyz);
	printf("%.1f %.1f\n\n\r",icm20948->pitch,icm20948->roll);
*/
}


//========================================================================================= MAG

void Mag_Writebyte(ICM20948 * icm20948,uint8_t register_address,uint8_t data){
	uint8_t Trans[2]={register_address, data};
	I2C_Transmit(&icm20948->i2c,icm20948->magneto_address,Trans,2);
}
uint8_t Mag_Readbyte(ICM20948 * icm20948,uint8_t register_address){
	uint8_t Trans[1]={register_address};
	uint8_t Receive[1];
	I2C_Transmit(&icm20948->i2c,icm20948->magneto_address,Trans,1);
	I2C_Receive(&icm20948->i2c,icm20948->magneto_address,Receive,1);
	return Receive[0];
}



uint8_t AK_Company_ID(ICM20948* icm20948){
	return Mag_Readbyte(icm20948,0x00);
}

uint8_t AK_Device_ID(ICM20948* icm20948){
	return Mag_Readbyte(icm20948,0x01);
}

void init_AK09916(ICM20948* icm20948){
	LL_mDelay(300);
	Mag_Writebyte(icm20948,AK09916_CNTL2,0x08);	//mode4
}

void AK_ReadData(ICM20948* icm20948,float* data,int16_t* data_int){
	uint8_t receive[6];
	int16_t temp[3];
	uint8_t reg;
	if(!(Mag_Readbyte(icm20948,AK09916_ST1) & 0x01)){
		return;
	}

	reg=AK09916_XOUT_L;
	I2C_Transmit(&icm20948->i2c,icm20948->gyro_address,&reg,1);
	I2C_Receive(&icm20948->i2c,icm20948->magneto_address,receive,6);

	temp[0]=(int16_t)(receive[1]<<8 | receive[0]);
	temp[1]=(int16_t)(receive[3]<<8 | receive[2]);
	temp[2]=(int16_t)(receive[5]<<8 | receive[4]);
	//data[0]=((float)temp[0])/6.66;
	//data[1]=((float)temp[1])/6.66;
	//data[2]=((float)temp[2])/6.66;
	data_int[0]=temp[0];
	data_int[1]=temp[1];
	data_int[2]=temp[2];
	Mag_Readbyte(icm20948,AK09916_ST2);
}

