/*
 * ICM20948_reg.h
 *
 *  Created on: Jun 1, 2020
 *      Author: perSec
 */

#ifndef INC_ICM20948_REG_H_
#define INC_ICM20948_REG_H_

//Magnetometer Registers
#define AK09916_ADDRESS  0x0C
#define WHO_AM_I_AK09916 0x01 // (AKA WIA2) should return 0x09
#define AK09916_ST1      0x10  // data ready status bit 0
#define AK09916_XOUT_L   0x11  // data
#define AK09916_XOUT_H   0x12
#define AK09916_YOUT_L   0x13
#define AK09916_YOUT_H   0x14
#define AK09916_ZOUT_L   0x15
#define AK09916_ZOUT_H   0x16
#define AK09916_ST2      0x18  // Data overflow bit 3 and data read error status bit 2
#define AK09916_CNTL2    0x31
#define AK09916_CNTL3    0x32  // Normal (0), Reset (1)

// ICM-20948

// USER BANK 0 REGISTER MAP
#define WHO_AM_I_ICM20948  0x00 // Should return 0xEA
#define USER_CTRL          0x03  // Bit 7 enable DMP, bit 3 reset DMP
#define LP_CONFIG		   0x05 // Not found in MPU-9250
#define PWR_MGMT_1         0x06 // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x07
#define INT_PIN_CFG        0x0F
#define INT_ENABLE         0x10
#define INT_ENABLE_1	   0x11 // Not found in MPU-9250
#define INT_ENABLE_2	   0x12 // Not found in MPU-9250
#define INT_ENABLE_3	   0x13 // Not found in MPU-9250
#define I2C_MST_STATUS     0x17
#define INT_STATUS         0x19
#define INT_STATUS_1	   0x1A // Not found in MPU-9250
#define INT_STATUS_2	   0x1B // Not found in MPU-9250
#define INT_STATUS_3	   0x1C // Not found in MPU-9250
#define DELAY_TIMEH		   0x28	// Not found in MPU-9250
#define DELAY_TIMEL		   0x29	// Not found in MPU-9250
#define ACCEL_XOUT_H       0x2D
#define ACCEL_XOUT_L       0x2E
#define ACCEL_YOUT_H       0x2F
#define ACCEL_YOUT_L       0x30
#define ACCEL_ZOUT_H       0x31
#define ACCEL_ZOUT_L       0x32
#define GYRO_XOUT_H        0x33
#define GYRO_XOUT_L        0x34
#define GYRO_YOUT_H        0x35
#define GYRO_YOUT_L        0x36
#define GYRO_ZOUT_H        0x37
#define GYRO_ZOUT_L        0x38
#define TEMP_OUT_H         0x39
#define TEMP_OUT_L         0x3A
#define EXT_SENS_DATA_00   0x3B
#define EXT_SENS_DATA_01   0x3C
#define EXT_SENS_DATA_02   0x3D
#define EXT_SENS_DATA_03   0x3E
#define EXT_SENS_DATA_04   0x3F
#define EXT_SENS_DATA_05   0x40
#define EXT_SENS_DATA_06   0x41
#define EXT_SENS_DATA_07   0x42
#define EXT_SENS_DATA_08   0x43
#define EXT_SENS_DATA_09   0x44
#define EXT_SENS_DATA_10   0x45
#define EXT_SENS_DATA_11   0x46
#define EXT_SENS_DATA_12   0x47
#define EXT_SENS_DATA_13   0x48
#define EXT_SENS_DATA_14   0x49
#define EXT_SENS_DATA_15   0x4A
#define EXT_SENS_DATA_16   0x4B
#define EXT_SENS_DATA_17   0x4C
#define EXT_SENS_DATA_18   0x4D
#define EXT_SENS_DATA_19   0x4E
#define EXT_SENS_DATA_20   0x4F
#define EXT_SENS_DATA_21   0x50
#define EXT_SENS_DATA_22   0x51
#define EXT_SENS_DATA_23   0x52
#define FIFO_EN_1          0x66
#define FIFO_EN_2          0x67 // Not found in MPU-9250
#define FIFO_RST		   0x68 // Not found in MPU-9250
#define FIFO_MODE		   0x69 // Not found in MPU-9250
#define FIFO_COUNTH        0x70
#define FIFO_COUNTL        0x71
#define FIFO_R_W           0x72
#define DATA_RDY_STATUS	   0x74 // Not found in MPU-9250
#define FIFO_CFG		   0x76 // Not found in MPU-9250
#define REG_BANK_SEL	   0x7F // Not found in MPU-9250

// USER BANK 1 REGISTER MAP
#define SELF_TEST_X_GYRO  			0x02
#define SELF_TEST_Y_GYRO  			0x03
#define SELF_TEST_Z_GYRO  			0x04
#define SELF_TEST_X_ACCEL 			0x0E
#define SELF_TEST_Y_ACCEL 			0x0F
#define SELF_TEST_Z_ACCEL 			0x10
#define XA_OFFSET_H       			0x14
#define XA_OFFSET_L       			0x15
#define YA_OFFSET_H       			0x17
#define YA_OFFSET_L       			0x18
#define ZA_OFFSET_H       			0x1A
#define ZA_OFFSET_L       			0x1B
#define TIMEBASE_CORRECTION_PLL		0x28

// USER BANK 2 REGISTER MAP
#define GYRO_SMPLRT_DIV        	0x00 // Not found in MPU-9250
#define GYRO_CONFIG_1      		0x01 // Not found in MPU-9250
#define GYRO_CONFIG_2      		0x02 // Not found in MPU-9250
#define XG_OFFSET_H       		0x03  // User-defined trim values for gyroscope
#define XG_OFFSET_L       		0x04
#define YG_OFFSET_H       		0x05
#define YG_OFFSET_L       		0x06
#define ZG_OFFSET_H       		0x07
#define ZG_OFFSET_L       		0x08
#define ODR_ALIGN_EN			0x09 // Not found in MPU-9250
#define ACCEL_SMPLRT_DIV_1     	0x10 // Not found in MPU-9250
#define ACCEL_SMPLRT_DIV_2     	0x11 // Not found in MPU-9250
#define ACCEL_INTEL_CTRL		0x12 // Not found in MPU-9250
#define ACCEL_WOM_THR			0x13 // Not found in MPU-9250 (could be WOM_THR)
#define ACCEL_CONFIG      		0x14
#define ACCEL_CONFIG_2     		0x15 // Not found in MPU-9250 (could be ACCEL_CONFIG2)
#define FSYNC_CONFIG			0x52 // Not found in MPU-9250
#define TEMP_CONFIG				0x53 // Not found in MPU-9250
#define MOD_CTRL_USR			0x54 // Not found in MPU-9250

// USER BANK 3 REGISTER MAP
#define I2C_MST_ODR_CONFIG		0x00 // Not found in MPU-9250
#define I2C_MST_CTRL       		0x01
#define I2C_MST_DELAY_CTRL 		0x02
#define I2C_SLV0_ADDR      		0x03
#define I2C_SLV0_REG       		0x04
#define I2C_SLV0_CTRL      		0x05
#define I2C_SLV0_DO        		0x06
#define I2C_SLV1_ADDR      		0x07
#define I2C_SLV1_REG       		0x08
#define I2C_SLV1_CTRL      		0x09
#define I2C_SLV1_DO        		0x0A
#define I2C_SLV2_ADDR      		0x0B
#define I2C_SLV2_REG       		0x0C
#define I2C_SLV2_CTRL      		0x0D
#define I2C_SLV2_DO        		0x0E
#define I2C_SLV3_ADDR      		0x0F
#define I2C_SLV3_REG       		0x10
#define I2C_SLV3_CTRL      		0x11
#define I2C_SLV3_DO        		0x12
#define I2C_SLV4_ADDR      		0x13
#define I2C_SLV4_REG       		0x14
#define I2C_SLV4_CTRL      		0x15
#define I2C_SLV4_DO        		0x16
#define I2C_SLV4_DI        		0x17


// Using the ICM-20948 breakout board, ADO is set to 1
// Seven-bit device address is 1000100 for ADO = 0 and 1000101 for ADO = 1
#define ADO 0
#if ADO
#define ICM20948_ADDRESS 0x69  // Device address when ADO = 1
#else
#define ICM20948_ADDRESS 0x68  // Device address when ADO = 0
#define AK09916_ADDRESS  0x0C   // Address of magnetometer
#endif // AD0

#define READ_FLAG 0x80



#endif /* INC_ICM20948_REG_H_ */
