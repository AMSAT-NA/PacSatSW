/*
 * MRAMManagement.h
 *
 *  Created on: Jun 5, 2019
 *      Author: bfisher
 */

#ifndef UTILITIES_INC_NONVOLMANAGEMENT_H_
#define UTILITIES_INC_NONVOLMANAGEMENT_H_

#include "MRAMmap.h"

int SetupMRAM(void);
void IHUInitSaved(void);

bool SetupMRAMWOD(void);

bool ReadMRAMBoolState(int index);
void WriteMRAMBoolState(int index,bool state);
void WriteMRAM2BoolState(int index1,bool state1,int index2,bool state2);

void WriteMRAMWODFreq(uint16_t freq);
uint16_t ReadMRAMWODFreq(void);

void WriteMRAMWODMaxFileSize(uint16_t size);
uint16_t ReadMRAMWODMaxFileSize(void);

void WriteMRAMResets(uint16_t resets);
uint16_t ReadMRAMResets(void);

void WriteMRAMTimestampResets(uint16_t resets);
uint16_t ReadMRAMTimestampResets(void);


void WriteMRAMSecondsOnOrbit(uint32_t seconds);
uint32_t ReadMRAMSecondsOnOrbit(void);

void WriteMinMaxResetSeconds(uint32_t seconds);
uint32_t ReadMinMaxResetSeconds(void);
void WriteMinMaxResetEpoch(uint16_t epoch);
uint32_t ReadMinMaxResetEpoch(void);

void WriteMRAMSecondsRemainingSciMode(uint32_t seconds);
uint32_t ReadMRAMSecondsRemainingSciMode(void);

void WriteMRAMEnterAutosafe(uint16_t resets);
uint16_t ReadMRAMEnterAutosafe(void);

void WriteMRAMExitAutosafe(uint16_t resets);
uint16_t ReadMRAMExitAutosafe(void);

void WriteMRAMTimeout(TimeoutType type,uint32_t seconds);
uint32_t ReadMRAMTimeout(TimeoutType type);


uint32_t ReadMRAMCountdownAfterRelease(void);
void WriteMRAMCountdownAfterRelease(uint32_t);

uint32_t ReadMRAMDCTDriveHighPower(void);
void WriteMRAMDCTDriveHighPower(uint32_t);
uint32_t ReadMRAMDCTDriveLowPower(void);
void WriteMRAMDCTDriveLowPower(uint32_t);

uint32_t ReadMRAMFreq(uint8_t devnum);
void WriteMRAMFreq(uint8_t devnum, uint32 freq);
enum radio_modulation ReadMRAMModulation(uint8_t devnum);
void WriteMRAMModulation(uint8_t devnum, enum radio_modulation mod);

void WriteMRAMVersionNumber(void);
bool CheckMRAMVersionNumber(void);

void WriteMRAMHighestFileNumber(uint32_t seconds);
uint32_t ReadMRAMHighestFileNumber(void);

void WriteMRAMPBStatusFreq(uint16_t freq);
uint16_t ReadMRAMPBStatusFreq(void);
void WriteMRAMFTL0StatusFreq(uint16_t freq);
uint16_t ReadMRAMFTL0StatusFreq(void);
void WriteMRAMTelemFreq(uint16_t freq);
uint16_t ReadMRAMTelemFreq(void);
void WriteMRAMTimeFreq(uint16_t freq);
uint16_t ReadMRAMTimeFreq(void);
void WriteMRAMExpFreq(uint16_t freq);
uint16_t ReadMRAMExpFreq(void);
void WriteMRAMExpMaxFileSize(uint16_t size);
uint16_t ReadMRAMExpMaxFileSize(void);

void WriteMRAMReceiverMode(uint8_t rxNum,uint8_t val);
uint8_t ReadMRAMReceiverMode(uint8_t rxNum);

#define ReadMRAMSafeMode() ReadMRAMBoolState(StateSafeMode)
#define WriteMRAMAutoSafe(value) WriteMRAMBoolState(StateAutoSafe,value)
#define ReadMRAMAutoSafe() ReadMRAMBoolState(StateAutoSafe)
#define WriteMRAMAllowAutoSafe(value) WriteMRAMBoolState(StateAutoSafeAllow,value)
#define WriteMRAMCommandReceived(value) WriteMRAMBoolState(StateCommandReceived,value)


#endif /* UTILITIES_INC_NONVOLMANAGEMENT_H_ */
