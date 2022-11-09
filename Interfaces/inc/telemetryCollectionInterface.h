/*
 * telemetryCollectionInterface.h
 *
 *  Created on: Jun 22, 2013
 *      Author: Mike McCann KB2GHZ
 */

#ifndef TELEMETRYCOLLECTIONINTERFACE_H_
#define TELEMETRYCOLLECTIONINTERFACE_H_

#include <stdint.h>
#include "downlink.h"
#include "TelemetryCollection.h"
#include "MET.h"
#include "MRAMmap.h"
//#include "CANQueueManagement.h"

/*
 * This structure is filled when data is collected by the TelemetryCollection task
 */

void ClearMinMax ( void );
void CreatePayload(int, anyPayload_t*);
void CreateHeader(int frameType, header_t *);
int getMinMaxResetCount(void);
void loadCommonFields2(commonRtWodPayload_t *p, WODHkMRAM_t *pRT);
void loadCommonFields(commonRtMinmaxWodPayload_t *p, WODHkMRAM_t *pRT);
int16_t GetStandbyRSSI(void);

//void FillCANFrames(CANHealthPayload_t *buffer, int size, CANQType type);


#define nextBuffer(i)  ((i==0) ? 1: 0)

#define ICR_GOLFT
typedef struct {
	/* This is in channel order.  Unfortunately
	 * The prototype, EM, and FM ICR cards all have
	 * different order!
	 */
#ifdef ICR_PROTOTYPE
	uint16_t ReflectedPower; /* channel 0* */
	uint16_t Regulated3V;
	uint16_t RegulatedProt3V;
	uint16_t Regulated2dot5V;
	uint16_t RegulatedProt2dot5V;
	uint16_t spare1;
	uint16_t spare2;
	uint16_t SensorPow;
#elif defined (ICR_EM)
	uint16_t ReflectedPower; /* channel 0* */
	uint16_t RegulatedProt2dot5V;
	uint16_t RegulatedProt3V;
	uint16_t Regulated3V;
	uint16_t Regulated2dot5V;
	uint16_t spare1;
	uint16_t spare2;
	uint16_t SensorPow;
#elif defined (ICR_FM)
    uint16_t ReflectedPower; /* channel 0* */
    uint16_t RegulatedProt3V;
    uint16_t Regulated3V;
    uint16_t Regulated2dot5V;
    uint16_t RegulatedProt2dot5V;
    uint16_t VBattUnreg;   //VBattUnregSys, mainly for DUBSat
    uint16_t spare;
    uint16_t SensorPow;
#elif defined (ICR_HUSKYSat)
#define I2C_ICR_PORT I2C1
    uint16_t ReflectedPower; /* channel 0* */
    uint16_t RegulatedProt3V;
    uint16_t DistBoardStat;
    uint16_t Regulated2dot5V;
    uint16_t MinusXSolarPanel;
    uint16_t XSolarPanel;
    uint16_t YSolarPanel;
    uint16_t Vsensors;
#elif defined (ICR_GOLFT)
    uint16_t ReflectedPower; /* channel 0* */
    uint16_t RegulatedProt3V;
    uint16_t ICRTemp;
    uint16_t TxTemp;
    uint16_t VGA;
    uint16_t RxPowerOn;
    uint16_t RSSI;
    uint16_t ForwardPower;
#else
#error Must define ICR type
#endif
} ICRinfo_t;
#define CIU_GOLFT_PROTO
#if defined(CIU_GOLFT_PROTO)
typedef struct _SolInfo_t {
    uint16_t XVolts; /* channel 0* */
    uint16_t YVolts;
    uint16_t MinusXVolts;
    uint16_t MinusYVolts;
    uint16_t XTemp;
    uint16_t YTemp;
    uint16_t MinusYTemp;
    uint16_t MinusXTemp;
} SolInfo_t;
typedef struct _CSSInfo_t {
    uint16_t CSS1; //Channel 0
    uint16_t CSS2; //Channel 1
    uint16_t CSS3; //Channel 2
    uint16_t CSS4; //Channel 3
    uint16_t CSS5; //Channel 4
    uint16_t CSS6; //Channel 5
    uint16_t CSS7; //Channel 6
    uint16_t CSS8; //Channel 7
} CSSInfo_t;

#endif


#endif /* TELEMETRYCOLLECTIONINTERFACE_H_ */
