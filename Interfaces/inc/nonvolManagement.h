/*
 * MRAMManagement.h
 *
 *  Created on: Jun 5, 2019
 *      Author: bfisher
 */

#ifndef UTILITIES_INC_NONVOLMANAGEMENT_H_
#define UTILITIES_INC_NONVOLMANAGEMENT_H_

#include "MRAMmap.h"

int MRAMInit(void);
void IHUInitSaved(void);

bool MRAMInitWOD(void);

bool ReadMRAMBoolState(int index);
void WriteMRAMBoolState(int index,bool state);
void WriteMRAM2BoolState(int index1,bool state1,int index2,bool state2);

void WriteMRAMWODHkStoreIndex(uint32_t index);
uint32_t ReadMRAMWODHkStoreIndex(void);

void WriteMRAMWODSciStoreIndex(uint32_t index);
uint32_t ReadMRAMWODSciStoreIndex(void);

void WriteMRAMWODHkDownlinkIndex(uint32_t index);
uint32_t ReadMRAMWODHkDownlinkIndex(void);

void WriteMRAMWODSciDownlinkIndex(uint32_t index);
uint32_t ReadMRAMWODSciDownlinkIndex(void);

void WriteMRAMWODRagStoreIndex(uint32_t index);
uint32_t ReadMRAMWODRagStoreIndex(void);

void WriteMRAMWODRagDownlinkIndex(uint32_t index);
uint32_t ReadMRAMWODRagDownlinkIndex(void);


void WriteMRAMWODFreq(uint16_t freq);
uint16_t ReadMRAMWODFreq(void);

void WriteMRAMWODSaved(uint16_t size);
uint16_t ReadMRAMWODSaved(void);

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
uint32_t ReadMRAMTelemFreq(void);
void WriteMRAMTelemFreq(uint32_t);
uint32_t ReadMRAMCommandFreq(void);
void WriteMRAMCommandFreq(uint32_t);

void WriteMRAMVersionNumber(void);
bool CheckMRAMVersionNumber(void);


int mram_test1(int size);
int mram_test2(int size);

#define ReadMRAMSafeMode() ReadMRAMBoolState(StateSafeMode)
#define WriteMRAMAutoSafe(value) WriteMRAMBoolState(StateAutoSafe,value)
#define ReadMRAMAutoSafe() ReadMRAMBoolState(StateAutoSafe)
#define WriteMRAMAllowAutoSafe(value) WriteMRAMBoolState(StateAutoSafeAllow,value)
#define WriteMRAMCommandReceived(value) WriteMRAMBoolState(StateCommandReceived,value)


#endif /* UTILITIES_INC_NONVOLMANAGEMENT_H_ */
