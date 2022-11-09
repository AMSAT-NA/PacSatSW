/*
 * MPU9250-Gyro.c
 *
 *  Created on: Feb 20, 2018
 *      Author: Burns Fisher
 *
 */
#include <pacsat.h>
#include "FreeRTOS.h"
#include "os_task.h"


#include "MPU9250-Gyro.h"
#include "mpu9250-regs.h"

uint8_t	dis_i2c=G9250_CMD_I2CDIS;
#define GYRO_REG_ACCEL_XOUT_H 59
#define GYRO_REG_ACCEL_XOUT_L 60
#define GYRO_REG_ACCEL_YOUT_H 61
#define GYRO_REG_ACCEL_YOUT_L 62
#define GYRO_REG_ACCEL_ZOUT_H 63
#define GYRO_REG_ACCEL_ZOUT_L 64

#define GYRO_TEMP_SENSITIVITY 333.87
#define GYRO_REG_TEMP_H 65
#define GYRO_REG_TEMP_L 66

#define GYRO_REG_SPIN_XOUT_H 67
#define GYRO_REG_SPIN_XOUT_L 68
#define GYRO_REG_SPIN_YOUT_H 69
#define GYRO_REG_SPIN_YOUT_L 70
#define GYRO_REG_SPIN_ZOUT_H 71
#define GYRO_REG_SPIN_ZOUT_L 72


#define GYRO_REG_I2C_MASTER_CTRL 36
#define GYRO_REG_I2C_SLV0_ADDR 37
#define GYRO_REG_I2C_SLV0_REGNUM 38
#define GYRO_REG_I2C_SLV0_CTRL 39
#define GYRO_REG_I2C_MASTER_STATUS 54
#define GYRO_REG_INT_CONFIG 55
#define GYRO_REG_I2C_EXT_DATA0 73

#define GYRO_MAGNETOMETER_READ_ADDRESS 0x8c   /*Address c in the bottom.  Top bit set for read */
#define GYRO_MAG_I2C_SLV_CTRL 0x86 /*Enable and read 6 bytes*/
#define GYRO_MAG_PASSTHROUGH 0x22

#define MAGNETOMETER_XH_ADDRESS 3

/*
 * I think we are going to get this data over CAN from the LIHU
 */

#if 0
bool GyroWriteOneRegister(uint8_t regAddress,uint8_t value){
	uint8_t val = value;
	return SPISendCommand(GyroDev, regAddress | G9250_WRITE,1,
			&val,1,
			(void *)SPI_NONE,SPI_NONE);
}
bool GyroReadRegisters(uint8_t regAddress, uint8_t *values, uint16_t length){
	return SPISendCommand(GyroDev, regAddress | G9250_READ,1,
			(void *)SPI_NONE,SPI_NONE,
			values,length);

}
void ReverseAndOffsetBytes(uint8_t *bytes,int offset){
	/*
	 * Treat incoming pairs of bytes as 16-bit words
	 * and reverse the bytes in them.  Then turn them
	 * from signed words to unsigned (offset by 32768)
	 */
	uint16_t temp;
	uint32_t temp32;
	 int16_t   *signedWord = ( int16_t *)bytes;
	uint16_t *unsignedWord = (uint16_t *)signedWord;
	temp = bytes[0];
	bytes[0] = bytes[1];
	bytes[1] = temp;
	temp32 = *signedWord; //Make it plenty big
	temp32 += offset;     // Offset by the specified amount
	*unsignedWord = temp32; // Hand it back unsigned.
}

#endif

uint16_t GyroGetTemp(void){
    return 0;
}
bool GyroReadSpin(uint16_t *values){
    return true;
}
bool GyroReadAccel(uint16_t *values){
    return true;
}
bool GyroReadMag(uint16_t *values){
    return true;
}

bool GyroInit(void){
    return true;
}

#if 0
uint16_t GyroGetTemp(void){
	int16_t signedVal16;
	int32_t signedVal32;
	GyroReadRegisters(MPUREG_TEMP_OUT_H,(uint8_t *)&signedVal16,2);
	ReverseAndOffsetBytes((uint8_t *)&signedVal16,0);
	signedVal32 = signedVal16; // Get it into 32 bits for the next few lines of fixed point.
	/*
	 * Note:  The following line is really using fixed point arithmetic to do the equivalent
	 * of signedVal/(G_T_S/10) which ends up being an integer value in 10ths of a degree relative
	 * to 21C.
	 */
	signedVal32 = (signedVal32*1000)/((uint32_t)GYRO_TEMP_SENSITIVITY*100.);
	signedVal32 +=610;  // Adjust to offset from -40C in tenths so we can treat it as unsigned positive.

	return (uint16_t)signedVal32 & 0xFFFF;
}
#define OFFSET16 32768
bool GyroReadSpin(uint16_t *values){
	bool retval;
	retval = GyroReadRegisters(MPUREG_GYRO_XOUT_H,(uint8_t *)values,6);
	ReverseAndOffsetBytes((uint8_t *)&values[0],OFFSET16);
	ReverseAndOffsetBytes((uint8_t *)&values[1],OFFSET16);
	ReverseAndOffsetBytes((uint8_t *)&values[2],OFFSET16);
	return retval;
}
bool GyroReadAccel(uint16_t *values){
	bool retval;
	retval = GyroReadRegisters(MPUREG_ACCEL_XOUT_H,(uint8_t *)values,6); // Read 6 bytes (hi and low for each dimension)
	ReverseAndOffsetBytes((uint8_t *)&values[0],OFFSET16); //Reverse each dimension to standard order
	ReverseAndOffsetBytes((uint8_t *)&values[1],OFFSET16);
	ReverseAndOffsetBytes((uint8_t *)&values[2],OFFSET16);
	return retval;

}
bool GyroReadMag(uint16_t *values){
	bool readingOk;
	int16_t localVal[4];

	GyroWriteOneRegister(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
	GyroWriteOneRegister(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
	GyroWriteOneRegister(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 6 bytes from the magnetometer
	vTaskDelay(2);
	GyroReadRegisters(MPUREG_EXT_SENS_DATA_00,(uint8_t *)localVal,7);
	readingOk = ((localVal[3] & AK8963_ST2_ERROR) == 0);
	if(readingOk){
		int32_t temp;
		/*
		 * Copy these into the 16-bit integers passed to us, and then treat them as signed (which they were
		 * originally) and offset them so that 0 = -32768 and 65535 is +32767
		 */
		temp = localVal[0];
		values[0] = (uint16_t)(temp + OFFSET16);
		temp = localVal[1];
		values[1] = (uint16_t)(temp + OFFSET16);
		temp = localVal[2];
		values[2] = (uint16_t)(temp + OFFSET16);
	}
	return readingOk;
}
bool GyroReadI2CStatus(uint8_t *value){
	bool retval;
	retval = GyroReadRegisters(GYRO_REG_I2C_MASTER_STATUS,value,1); // Read 6 bytes (hi and low for each dimension)
    return retval;
}
bool GyroInit(void){
	int i;
#define MPU_InitRegNum 15
	uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
			{BIT_H_RESET, MPUREG_PWR_MGMT_1},        // Reset Device
			{0x01, MPUREG_PWR_MGMT_1},               // Clock Source
			{0x00, MPUREG_PWR_MGMT_2},               // Enable Acc & Gyro
			//{my_low_pass_filter, MPUREG_CONFIG},     // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
			{BITS_FS_250DPS, MPUREG_GYRO_CONFIG},    // +-250dps
			{BITS_FS_2G, MPUREG_ACCEL_CONFIG},       // +-2G
			//{my_low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
			{0x12, MPUREG_INT_PIN_CFG},      //
			//{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
			//{0x20, MPUREG_USER_CTRL},      // Enable AUX
			{0x30, MPUREG_USER_CTRL},        // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
			{0x0D, MPUREG_I2C_MST_CTRL},     // I2C configuration multi-master  IIC 400KHz

			{AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  // Set the I2C slave addres of AK8963 and set for write.
			//{0x09, MPUREG_I2C_SLV4_CTRL},
			//{0x81, MPUREG_I2C_MST_DELAY_CTRL}, // Enable I2C delay

			{AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
			{0x01, MPUREG_I2C_SLV0_DO},   // Reset AK8963
			{0x81, MPUREG_I2C_SLV0_CTRL}, // Enable I2C and set 1 byte

			{AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
#ifdef AK8963FASTMODE
			{0x16, MPUREG_I2C_SLV0_DO},   // Register value to 100Hz continuous measurement in 16bit
#else
			{0x12, MPUREG_I2C_SLV0_DO},   // Register value to 8Hz continuous measurement in 16bit
#endif
			{0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte

	};

	for(i = 0; i < MPU_InitRegNum; i++) {
		GyroWriteOneRegister(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
		vTaskDelay(2);  // I2C must slow down the write speed, otherwise it won't work
	}
	return true;
}
uint8_t MagWhami(void){
    uint8_t response;
    GyroWriteOneRegister(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    GyroWriteOneRegister(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    GyroWriteOneRegister(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer
    vTaskDelay(2);
    GyroReadRegisters(MPUREG_EXT_SENS_DATA_00,&response,1);
    //response = WriteReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);    //Read I2C
    //ReadRegs(MPUREG_EXT_SENS_DATA_00,response,1);
    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C

return response;
}
#endif
