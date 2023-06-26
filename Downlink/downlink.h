/*
 * downlink.h
 *
 *  Created on: May 25, 2013
 *      Author: Mike McCann KB2GHZ
 *      Updated on 5/29/2013 per Downlink Specification V1.3
 *		Updated on 6/7/2013 per Downlink Specification V1.31
 *		Updated on 6/14/2013 to add DownlinkErrors_t structure to min values frame
 *		Added packed attribute to the header structures on 6/21/2013
 *		All temperature fields are now 12-bit 12/1/2013
 *		Updated on 1/18/2014 per Downlink Specification V1.53
 *		Updated on 2/14/2014 per Downlink Specification V1.54
 *		Updated on 3/23/2014 per Downlink Specification V1.55
 *		Updated on 3/27/2014 per Downlink Specification V1.56
 *		Updated on 6/11/2014 per Downlink Specification V1.58
 *
 *  Revised 26 May 2023 for PACSAT
 *
 */


#ifndef DOWNLINK_H_
#define DOWNLINK_H_
// This structure is used in some of the includes below

/* Define the architecture to generate an error if we have the wrong version included in the downlink
 * Use LIHU for STM32 Little Endian processor.  Use RTIHU for big endian TMS570 or similar
 */
#define RTIHU
#define EXTENDED_ID 6 // If this is in the low 3 bits of the satellite ID, we use the new format header
#define SAT_ID_SHIFT 3 //To make an 8-bit satellite ID, we shift the SAT ID by this and OR in the extended ID.
#define MAJOR_VERSION_SHIFT 4 //To make 8-bit version, shift the major by this much and or the minor.

#include "stdint.h"
#include "errors.h"
#include "commonDownlink.h"
#include "common2Downlink.h"
#include "maxDownlink.h"
#include "minDownlink.h"
#include "realtimeDownlink.h"
#include "wodSpecificDownlink.h"
#include "downlinkVersion.h"
#include "headerDownlink.h"
#include "rt1ErrorsDownlink.h"


/* Real-Time Telemetry Frame (Type-1) */
typedef struct __attribute__((__packed__)) {
	commonRtMinmaxWodPayload_t common; 		/* common telemetry fields */
	commonRtWodPayload_t common2;
	realtimeSpecific_t realTimeData;		/* fields unique to a type 1 payload */
    rt1Errors_t primaryErrors;  // This structure is also used for local regardless of primary or secondary
} realTimePayload_t;

/* Maximum Values Telemetry Frame (Type-2) */
typedef struct __attribute__((__packed__)) {
	commonRtMinmaxWodPayload_t common;		/* common telemetry fields */
	maxSpecific_t MaxValuesData;	/* fields unique to a type 2 payload */
} maxValuesPayload_t;

/* Minimum Values Telemetry Frame (Type-3) */
typedef struct __attribute__((__packed__)) {
	commonRtMinmaxWodPayload_t common;		/* common telemetry fields */
	minSpecific_t MinValuesData;	/* fields unique to a type 3 payload */
} minValuesPayload_t;


typedef struct __attribute__((__packed__)){
	commonRtMinmaxWodPayload_t common;
    commonRtWodPayload_t common2;
	wodSpecific_t wodInfo;
} WODHousekeepingPayload_t;


/* define symbols for the telemetry payload types */
#define HEADER_PAYLOAD			0		/* Header */
#define RT_HK_PAYLOAD			1		/* Real-Time Telemetry */
#define MAX_VALS_PAYLOAD		2		/* Maximum Values Telemetry */
#define MIN_VALS_PAYLOAD		3		/* Minimum Values Telemetry */
#define WOD_HK_PAYLOAD			4
#define RT_RAD_PAYLOAD         	5       /* Vanderbilt realtime data */
#define WOD_RAD_PAYLOAD			6
#define RT_RAGNAROK_PAYLOAD  	7		/**/
#define WOD_RAGNAROK_PAYLOAD	8
#define FILLER_PAYLOAD          9
#define DIAGNOSTIC_PAYLOAD      10
#define MAX_LOW_SPEED_PAYLOADS  11		/* Number of low speed telemetry types */

/* Define symbols for telemetry frame types */

#define REALTIME_MIN_FRAME		0
#define REALTIME_MAX_FRAME		1
#define ALL_WOD1_FRAME			2
#define SAFE_DATA1_FRAME        3
#define SAFE_WOD_FRAME          4

#define TOTAL_FRAME_SIZE 648 /* This will add some zeros to fill the frame to this size after the payloads*/
#define CODE_WORDS_PER_FRAME 3 /* Every 223 bytes takes one code word */
#define MAX_PAYLOADS_PER_FRAME 9 /*Does not include CRC; includes header and filler*/
typedef struct __attribute__((__packed__)) { //Frame type 0
	header_t header;
	realTimePayload_t rtHealth;
	minValuesPayload_t minVals;
} realTimeMinFrame_t;
typedef struct __attribute__((__packed__)) {
	header_t header;
	realTimePayload_t rtHealth;
	maxValuesPayload_t maxVals;
} realTimeMaxFrame_t;
typedef struct __attribute__((__packed__)) {
	header_t header;
	WODHousekeepingPayload_t HKWod[3];
} allWOD1Frame_t;
typedef struct __attribute__((__packed__)) { //Frame type 3
    header_t header;
    realTimePayload_t rtHealth;
    minValuesPayload_t minVals;
    maxValuesPayload_t maxVals;
} safeData1Frame_t;

typedef struct __attribute__((__packed__)) { //Frame type 4
    header_t header;
    WODHousekeepingPayload_t HKWod[3];
} safeWODFrame_t;


#define DOWNLINK_IHU_TEMP_OFFSET 550 /* Subtrace this from ADC reading to put in 8-bit downlink */

#endif /* DOWNLINK_H_ */
