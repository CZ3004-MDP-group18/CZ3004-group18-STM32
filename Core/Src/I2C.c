/*
 * I2C.c
 *
 *  Created on: Jul 18, 2020
 *      Author: perSec
 */

#include "I2C.h"

void I2C_init(I2C_struct* I2C,I2C_TypeDef* I2C_type){
	I2C->I2C=I2C_type;
	I2C->i2c_busy=0;
	I2C->i2c_IsRx=0;
}


void I2C_Transmit(I2C_struct* I2C, uint8_t address, uint8_t* data,uint16_t size){
	if(I2C->i2c_busy==1){
		return;
	}
	I2C->i2c_busy=1;
	I2C->i2c_IsRx=0;
	LL_I2C_DisableDMAReq_RX(I2C->I2C);
	LL_I2C_DisableIT_TX(I2C->I2C);


	if(!LL_I2C_IsEnabled(I2C->I2C)){
		LL_I2C_Enable(I2C->I2C);
	}
	LL_I2C_DisableBitPOS(I2C->I2C);

	while(LL_I2C_IsActiveFlag_BUSY(I2C->I2C));

	LL_I2C_GenerateStartCondition(I2C->I2C);


	while( !LL_I2C_IsActiveFlag_SB(I2C->I2C) );
	LL_I2C_TransmitData8(I2C->I2C,address);



	while( !LL_I2C_IsActiveFlag_ADDR(I2C->I2C));
	LL_I2C_ClearFlag_ADDR(I2C->I2C);

	for(uint16_t i=0;i<size;i++){
		LL_I2C_TransmitData8(I2C->I2C,data[i]);
		while(!LL_I2C_IsActiveFlag_TXE(I2C->I2C));
	}
	while(!LL_I2C_IsActiveFlag_BTF(I2C->I2C));

	LL_I2C_GenerateStopCondition(I2C->I2C);
	I2C->i2c_busy=0;


}


void I2C_Receive(I2C_struct* I2C, uint8_t address, uint8_t* outputdata,uint16_t size){
	if(I2C->i2c_busy==1){
		return;
	}
	I2C->i2c_busy=1;
	I2C->i2c_IsRx=1;

	address |=(0x01);
	LL_I2C_DisableDMAReq_RX(I2C->I2C);
	LL_I2C_DisableIT_RX(I2C->I2C);

	if(!LL_I2C_IsEnabled(I2C->I2C)){
		LL_I2C_Enable(I2C->I2C);
	}
	LL_I2C_DisableBitPOS(I2C->I2C);


	while(LL_I2C_IsActiveFlag_BUSY(I2C->I2C));

	if(size==1)
		LL_I2C_AcknowledgeNextData(I2C->I2C,LL_I2C_NACK);
	else
		LL_I2C_AcknowledgeNextData(I2C->I2C,LL_I2C_ACK);



	LL_I2C_GenerateStartCondition(I2C->I2C);


	while( !LL_I2C_IsActiveFlag_SB(I2C->I2C) );
	LL_I2C_TransmitData8(I2C->I2C,address);

	while( !LL_I2C_IsActiveFlag_ADDR(I2C->I2C));
	LL_I2C_ClearFlag_ADDR(I2C->I2C);


	for(uint16_t i=0;i<size;i++){
		if(i==(size-1))
			LL_I2C_AcknowledgeNextData(I2C->I2C,LL_I2C_NACK);
		while(!LL_I2C_IsActiveFlag_RXNE(I2C->I2C));
		outputdata[i]=LL_I2C_ReceiveData8(I2C->I2C);
	}

	LL_I2C_GenerateStopCondition(I2C->I2C);

	I2C->i2c_busy=0;

}