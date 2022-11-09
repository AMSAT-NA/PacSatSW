/*
 * downlinkControl.h
 *
 *  Created on: Dec 29, 2012
 *      Author: Burns Fisher AMSAT-NA
 */

#ifndef DOWNLINK_CONTROL_H_
#define DOWNLINK_CONTROL_H_

#include "downlink.h"
#include "TelemEncoding.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "os_queue.h"
#include "os_timer.h"
#include "os_semphr.h"

#define BITS_PER_8b10b_WORD 10

#define LOW_SPEED_FRAME_SIZE sizeof(allFrames_t)
#define SYNC_SIZE 1
#define LOW_SPEED_PARITY_SIZE 32
/* Should be PARITY_BYTES_PER_CODE_WORD*CODE_WORDS_PER_LOW_SPEED_FRAME*/
#define LOW_SPEED_FEC_FRAME_SIZE (LOW_SPEED_FRAME_SIZE + LOW_SPEED_PARITY_SIZE*CODE_WORDS_PER_FRAME + 1)
#define LOW_SPEED_FEC_SYNC_FRAME_SIZE (SYNC_SIZE + LOW_SPEED_FRAME_SIZE + LOW_SPEED_PARITY_SIZE*CODE_WORDS_PER_FRAME + SYNC_SIZE)
// Remember that we have 4 extra bytes of CRC also
#define FEC_BUFFER_BYTES 2000 // Just make it plenty long.  LOW_SPEED_FEC_SYNC_FRAME_SIZE should be enough, but we have plenty
#define FEC_BUFFER_LONGWORDS ((FEC_BUFFER_BYTES+2)/3) /* How many longwords (4 byte words) at 3 10-bit words per longword*/


/*
 * These are the telemetry modes that the satellite can be in.  (Slightly different from the
 * statemachine state)
 */

typedef enum {
	 ModeHealth
	,ModeScience
	,ModeSafe
	,ModeAutosafe
	,ModeEclipseSafe
	,NumberOfModes
}TelemetryMode;

/* Here are the states in this state machine */

typedef enum {
    /*
     * There are a few special values here which are not really states.  Those are the ones that are 0 or
     * less.  These are treated specially by the "NewState" routine.
     */
    NoChange=0,     // No state change
    /* These are actual states */
    Health=1,       // In relay or transponder mode
    Safe=2,         // Safe mode, no carrier
    SafeBcon=3,     // Send safe mode carrier, and telemetry
    AutoSafe=4,
    AutoSfBcon=5,
    EclipSafe=6,
    EclipSfBcon=7,
    TransmitInhibit=8,
    ScienceMode=9,
    /* Followed by just numbers*/
    NUM_STATES,
    Unexpctd,
    TurnOnRx
}DownlinkStates;


/*
 * SAMPLES_PER_BIT is directly related to the downlink baud rate...it is defined in FoxConfig.h
 */

void DownlinkCtrlTask(void *pvParameters);

DownlinkStates GetCurrentDownlinkState(void);
void TransponderOn(void);
void TransponderOff(void);
void BeaconOn(void);
void BeaconOff(void);

void SendSafeModeMsg(bool autosafe);
void SendEclipseSafeModeMsg(void);
void SendHealthModeMsg(void);
void SendScienceModeMsg(int minutes);
void CommandInhibitTransmitting(void);
void CommandResumeTransmitting(void);

void ReprogramCallback (xTimerHandle handle);
bool TransmitterIsOnAndStable(void);
bool TransponderIsOn(void);
void EnableTransponder(bool toOn);
void SendCANPollMsgFromISR(void);
void SetAutosafeVoltages(void);
uint8_t GetAutosafeEnterVoltage(void);
uint8_t GetAutosafeExitVoltage(void);
bool GetSafeRfPowerLevel(void);
bool GetNormalRfPowerLevel(void);
void SetSafeRfPowerLevel(bool high);
void SetNormalRfPowerLevel(bool high);
void SetDCTDriveValue(uint16_t high, uint16_t low);
void DisableTelemetry(bool disable);

void ForceTransmittersOff(bool forceOff);
void OverrideUmbilical(bool override);
bool IsTxAllowedOn(void);
void InhibitTransponder(bool inhibit);

bool IsTransponderEnabled(void);
bool InCommandedSafeMode(void);
bool IsAutoSafeAllowed(void);
void SetAllowAutoSafe(bool allow);
bool InAutoSafeMode(void);
bool InEclipseSafeMode(void);
bool StartReceiveTimer(bool restart);
bool StandbyReceiverOk(void);


#endif /* DOWNLINK_CONTROL_H_ */
