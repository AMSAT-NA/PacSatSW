/*
 * ICRTelemetry.h
 *
 *  Created on: Jan 5, 2017
 *      Author: Burns Fisher W2BFJ
 *
 *      This implements the I2C interface to the Improved Command
 *      Receiver on Fox-1E and later
*/

#ifndef ICRTELEMETRY_H_
#define ICRTELEMETRY_H_
#include <pacsat.h>

#define PACBOARD_CPU_TEMP_ADDRESS (0x92 >> 1) /* 7-bit address mode */
#define PACBOARD_TX_TEMP_ADDRESS (0x90 >> 1) /* 7-bit address mode */
#define PACBOARD_TEMP_PORT I2C1

#define RTC_ADDRESS (0xD0 >> 1) /* 7-bit address mode */
#define RTC_PORT I2C1

#if 0
#define ICR_ADC_I2C_ADDRESS 0x4B //ICR voltages
#define ICR_ADC_I2C_PORT I2C2

#define CSS_ADC_I2C_ADDRESS 0x48 //Coarse Sun Sensors
#define CSS_ADC_I2C_PORT I2C1

#define SOLAR_ADC_I2C_ADDRESS 0x48 // Solar panel volts and temp
#define SOLAR_ADC_I2C_PORT I2C2

#define DEPLOY_GPIO_I2C_ADDRESS 0x77 // Deployables release and sense
#define DEPLOY_GPIO_I2C_PORT I2C2

#define EXP1_VUC_I2C_ADDRESS 0x2A // Vanderbilt VUC (EXP1)
#define EXP1_VUC_I2C_PORT I2C1


#define ICR_COLLECTION_REQUEST 0X01
#define ICR_STATUS_REQUEST 0x02
#define ICR_TELEMETRY_REQUEST 0x03
#endif
#endif /* ICRTELEMETRY_H_ */
