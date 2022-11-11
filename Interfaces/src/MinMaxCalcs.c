/*
 * MinMaxCalcs.c
 *
 *  Created on: Jul 27, 2021
 *      Author: bfisher
 */
#include <pacsat.h>
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "os_task.h"
#include "downlink.h"
#include "TelemetryCollection.h"
#include "MET.h"
#include "errors.h"
#include "MRAMmap.h"
#include "nonvol.h"
#include "gpioDriver.h"
#include "CANSupport.h"
#include "CANQueueManagement.h"
#include "ADS7828.h"
#include "inet.h"
#include "DownlinkControl.h"
#include "nonvolManagement.h"

static uint8_t  TelemetryResets=0; /* Could be extern, but not used anywhere else */


/* This function clears minimum and maximum telemetry values. */
void ClearMinMax() {
    MRAMMin_t minData;
    MRAMMax_t maxData;
    logicalTime_t currentTime;
    const MRAMmap_t *MRAMaddr = (MRAMmap_t *) 0;
    int NVstatus;
    bool printIt = (GPIORead(UmbilicalAttached)==1);
    TelemetryResets++;  /* Only 8 bits, so it can wrap.  That's ok.  The telemetry value is only 4 */
    getTimestamp(&currentTime); //Set min/max time to the current time

    #define printIf if(printIt)printf_
        printIf("Enter Clear Minmax...");


    minData.commonMin.Xspin = MRAM_MAX8;      //Offset=64
    minData.commonMin.Yspin = MRAM_MAX8;      //Offset=80
    minData.commonMin.Zspin = MRAM_MAX8;      //Offset=96
    minData.commonMin.Xaccel = MRAM_MAX8;      //Offset=112
    minData.commonMin.Yaccel = MRAM_MAX8;      //Offset=128
    minData.commonMin.Zaccel = MRAM_MAX8;      //Offset=144
    minData.commonMin.Xmag = MRAM_MAX8;      //Offset=160
    minData.commonMin.Ymag = MRAM_MAX8;      //Offset=176
    minData.commonMin.Zmag = MRAM_MAX8;      //Offset=192
    minData.commonMin.busV = MRAM_MAX8;       //Offset=112
    minData.commonMin.gTemp = MRAM_MAX8;       //Offset=124
    minData.commonMin.moduTemp = MRAM_MAX8;
    minData.commonMin.TxPAi = MRAM_MAX8;       //Offset=136
    minData.commonMin.TxTemp = MRAM_MAX8;       //Offset=148
    minData.commonMin.FwdPower = MRAM_MAX8;       //Offset=160
    minData.commonMin.rssi = MRAM_MAX8;       //Offset=172
    minData.commonMin.ReflectedPwr = MRAM_MAX8;       //Offset=208
    minData.commonMin.ICR3VProt = MRAM_MAX8;       //Offset=220
    minData.commonMin.ICRTemp = MRAM_MAX8;       //Offset=232
    minData.commonMin.MinusYVolts = MRAM_MAX8;       //Offset=244
    minData.commonMin.MinusXVolts = MRAM_MAX8;       //Offset=256
    minData.commonMin.XVolts = MRAM_MAX8;       //Offset=268
    minData.commonMin.YVolts = MRAM_MAX8;       //Offset=280
    minData.commonMin.MinusYTemp = MRAM_MAX8;       //Offset=292
    minData.commonMin.MinusXTemp = MRAM_MAX8;       //Offset=304
    minData.commonMin.XTemp = MRAM_MAX8;       //Offset=316
    minData.commonMin.YTemp = MRAM_MAX8;       //Offset=328
    minData.commonMin.LtVGACtl = MRAM_MAX8;       //Offset=340
    minData.commonMin.RT1Temp = MRAM_MAX8;       //Offset=352
    minData.commonMin.RT2Temp = MRAM_MAX8;       //Offset=364
    minData.commonMin.IHUcpuTemp = MRAM_MAX8;       //Offset=376
    minData.commonMin.CIU3V3_1 = MRAM_MAX8;       //Offset=388
    minData.commonMin.CIU3V3_2 = MRAM_MAX8;       //Offset=400
    minData.commonMin.CIUVsys = MRAM_MAX8;       //Offset=412
    minData.commonMin.Power10GHz = MRAM_MAX8;       //Offset=424
    minData.commonMin.ZynqTemp = MRAM_MAX8;
    minData.commonMin.SDRXspin = MRAM_MAX8;
    minData.commonMin.SDRYspin = MRAM_MAX8;
    minData.commonMin.SDRZspin = MRAM_MAX8;
    minData.commonMin.SDRXmag = MRAM_MAX8;
    minData.commonMin.SDRYmag = MRAM_MAX8;
    minData.commonMin.SDRZmag = MRAM_MAX8;
    minData.commonMin.SDRXaccel = MRAM_MAX8;
    minData.commonMin.SDRZaccel = MRAM_MAX8;

    minData.commonMin.SDRTcvrTemp = MRAM_MAX8;
    minData.commonMin.CIU12v = MRAM_MAX8;
    minData.commonMin.CSS1 = MRAM_MAX8;       //Offset=436
    minData.commonMin.CSS2 = MRAM_MAX8;       //Offset=448
    minData.commonMin.CSS3 = MRAM_MAX8;       //Offset=460
    minData.commonMin.CSS4 = MRAM_MAX8;       //Offset=472
    minData.commonMin.CSS5 = MRAM_MAX8;       //Offset=484
    minData.commonMin.CSS6 = MRAM_MAX8;       //Offset=496
    minData.commonMin.CSS7 = MRAM_MAX8;       //Offset=508
    minData.commonMin.CSS8 = MRAM_MAX8;       //Offset=520
    minData.commonMin.DCT1Power = MRAM_MAX8;
    minData.commonMin.DCT2Power = MRAM_MAX8;
    minData.commonMin.DCT1RSSI = MRAM_MAX8;
    minData.commonMin.DCT2RSSI = MRAM_MAX8;
    minData.commonMin.Current10GHz = MRAM_MAX8;
    minData.deltaMin.IHUresetCnt = currentTime.IHUresetCnt;
    minData.deltaMin.METcount = currentTime.METcount;
    minData.MinCrc = computeCRC((uint32_t *)&minData, CRClen(minData));
    printIf("Writing min to MRAM...");
    NVstatus = writeNV(&minData, sizeof(minData), ExternalMRAMData, (int) &(MRAMaddr->minData));
    if (NVstatus == false) { ReportError(MRAMwrite, false, ReturnAddr, (int)ClearMinMax);}

    printIf("Min done...");


    maxData.commonMax.Xspin = MRAM_MIN8;      //Offset=64
    maxData.commonMax.Yspin = MRAM_MIN8;      //Offset=80
    maxData.commonMax.Zspin = MRAM_MIN8;      //Offset=96
    maxData.commonMax.Xaccel = MRAM_MIN8;      //Offset=112
    maxData.commonMax.Yaccel = MRAM_MIN8;      //Offset=128
    maxData.commonMax.Zaccel = MRAM_MIN8;      //Offset=144
    maxData.commonMax.Xmag = MRAM_MIN8;      //Offset=160
    maxData.commonMax.Ymag = MRAM_MIN8;      //Offset=176
    maxData.commonMax.Zmag = MRAM_MIN8;      //Offset=192
    maxData.commonMax.busV = MRAM_MIN8;       //Offset=112
    maxData.commonMax.gTemp = MRAM_MIN8;       //Offset=124
    maxData.commonMax.moduTemp = MRAM_MIN8;
    maxData.commonMax.TxPAi = MRAM_MIN8;       //Offset=136
    maxData.commonMax.TxTemp = MRAM_MIN8;       //Offset=148
    maxData.commonMax.FwdPower = MRAM_MIN8;       //Offset=160
    maxData.commonMax.rssi = MRAM_MIN8;       //Offset=172
    maxData.commonMax.ReflectedPwr = MRAM_MIN8;       //Offset=208
    maxData.commonMax.ICR3VProt = MRAM_MIN8;       //Offset=220
    maxData.commonMax.ICRTemp = MRAM_MIN8;       //Offset=232
    maxData.commonMax.MinusYVolts = MRAM_MIN8;       //Offset=244
    maxData.commonMax.MinusXVolts = MRAM_MIN8;       //Offset=256
    maxData.commonMax.XVolts = MRAM_MIN8;       //Offset=268
    maxData.commonMax.YVolts = MRAM_MIN8;       //Offset=280
    maxData.commonMax.MinusYTemp = MRAM_MIN8;       //Offset=292
    maxData.commonMax.MinusXTemp = MRAM_MIN8;       //Offset=304
    maxData.commonMax.XTemp = MRAM_MIN8;       //Offset=316
    maxData.commonMax.YTemp = MRAM_MIN8;       //Offset=328
    maxData.commonMax.LtVGACtl = MRAM_MIN8;       //Offset=340
    maxData.commonMax.RT1Temp = MRAM_MIN8;       //Offset=352
    maxData.commonMax.RT2Temp = MRAM_MIN8;       //Offset=364
    maxData.commonMax.IHUcpuTemp = MRAM_MIN8;       //Offset=376
    maxData.commonMax.CIU3V3_1 = MRAM_MIN8;       //Offset=388
    maxData.commonMax.CIU3V3_2 = MRAM_MIN8;       //Offset=400
    maxData.commonMax.CIUVsys = MRAM_MIN8;       //Offset=412
    maxData.commonMax.Power10GHz = MRAM_MIN8;       //Offset=424
    maxData.commonMax.ZynqTemp = MRAM_MIN8;
    maxData.commonMax.SDRXspin = MRAM_MIN8;
    maxData.commonMax.SDRYspin = MRAM_MIN8;
    maxData.commonMax.SDRZspin = MRAM_MIN8;
    maxData.commonMax.SDRXmag = MRAM_MIN8;
    maxData.commonMax.SDRYmag = MRAM_MIN8;
    maxData.commonMax.SDRZmag = MRAM_MIN8;
    maxData.commonMax.SDRXaccel = MRAM_MIN8;
    maxData.commonMax.SDRZaccel = MRAM_MIN8;

    maxData.commonMax.SDRTcvrTemp = MRAM_MIN8;
    maxData.commonMax.CIU12v = MRAM_MIN8;
    maxData.commonMax.CSS1 = MRAM_MIN8;       //Offset=436
    maxData.commonMax.CSS2 = MRAM_MIN8;       //Offset=448
    maxData.commonMax.CSS3 = MRAM_MIN8;       //Offset=460
    maxData.commonMax.CSS4 = MRAM_MIN8;       //Offset=472
    maxData.commonMax.CSS5 = MRAM_MIN8;       //Offset=484
    maxData.commonMax.CSS6 = MRAM_MIN8;       //Offset=496
    maxData.commonMax.CSS7 = MRAM_MIN8;       //Offset=508
    maxData.commonMax.CSS8 = MRAM_MIN8;       //Offset=520
    maxData.commonMax.DCT1Power = MRAM_MIN8;
    maxData.commonMax.DCT2Power = MRAM_MIN8;
    minData.commonMin.DCT1RSSI = MRAM_MAX8;
    minData.commonMin.DCT2RSSI = MRAM_MAX8;
    maxData.commonMax.Current10GHz = MRAM_MIN8;

    maxData.deltaMax.IHUresetCnt = currentTime.IHUresetCnt;
    maxData.deltaMax.METcount = currentTime.METcount;
    maxData.MaxCrc = computeCRC((uint32_t *)&maxData, CRClen(maxData));
    printIf("Writing max to MRAM...");

    NVstatus = writeNV(&maxData, sizeof(maxData), ExternalMRAMData, (int) &(MRAMaddr->maxData));
    if (NVstatus == false) { ReportError(MRAMwrite, false, ReturnAddr, (int)ClearMinMax);}
    SetMramCRCGood();
    WriteMinMaxResetSeconds(currentTime.METcount);
    WriteMinMaxResetEpoch(currentTime.IHUresetCnt);

    printIf("Max done\n");
}


/* this function clears the logical time variables that record the last min and max
 * telemetry values found in the real-time telemetry
 */

int getMinMaxResetCount(){
    return TelemetryResets;
}


void MinMaxUpdate(WODHkMRAM_t *buffer) {
    bool NVstatus,updateMinCrc=false,updateMaxCrc=false;
    const MRAMmap_t *ptr = (MRAMmap_t *) 0;
    MRAMMin_t MinInfo; /* PSU's stored telemetry values read from MRAM */
    MRAMMax_t MaxInfo;
    uint32_t computedCRC;
    int updateMinCnt = 0,updateMaxCnt=0; /* count min/max changes */
    logicalTime_t currentTime;

    /* find new min/max values */
#define GetMinMax0OK(field) \
        if (buffer->wodHKPayload.common.field > MaxInfo.commonMax.field)\
        {MaxInfo.commonMax.field = buffer->wodHKPayload.common.field; updateMaxCnt++;}\
        if (buffer->wodHKPayload.common.field < MinInfo.commonMin.field)\
        {MinInfo.commonMin.field = buffer->wodHKPayload.common.field; updateMinCnt++;}
#define GetMinMax(field) \
        if(buffer->wodHKPayload.common.field != 0){ \
            if (buffer->wodHKPayload.common.field > MaxInfo.commonMax.field)\
            {MaxInfo.commonMax.field = buffer->wodHKPayload.common.field; updateMaxCnt++;}\
            if (buffer->wodHKPayload.common.field < MinInfo.commonMin.field)\
            {MinInfo.commonMin.field = buffer->wodHKPayload.common.field; updateMinCnt++;}\
        }

    getTimestamp(&currentTime); //Set min/max time to the current time

    /* read the min/max data from MRAM, verify stored CRC value */
    NVstatus=readNV(&MinInfo, sizeof(MRAMMin_t), ExternalMRAMData, (int) &(ptr->minData));
    if (NVstatus == false) { ReportError(MRAMread, false, ReturnAddr, (int)MinMaxUpdate);
    }
    computedCRC = computeCRC((uint32_t *) &MinInfo, sizeof(MinInfo)/4-1);
    if (computedCRC != MinInfo.MinCrc) {
        // Error--remember it.
        ReportError(MRAMcrc, false, ReturnAddr, (int)MinMaxUpdate);
        SetMramCRCError(); // Remember there was an error for telemetry
        updateMinCrc = true;  // Write a working CRC so we won't keep repeating this over and over.
    }


    NVstatus=readNV(&MaxInfo, sizeof(MRAMMax_t), ExternalMRAMData, (int) &(ptr->maxData));
    if (NVstatus == false) { ReportError(MRAMread, false, ReturnAddr, (int)MinMaxUpdate);
    }
    computedCRC = computeCRC((uint32_t *) &MaxInfo, sizeof(MaxInfo)/4-1);
    if (computedCRC != MaxInfo.MaxCrc) {
        // Error--remember it.
        ReportError(MRAMcrc, false, ReturnAddr, (int)MinMaxUpdate);
        SetMramCRCError(); // Remember there was an error for telemetry
        updateMaxCrc = true;  // Write a working CRC so we won't keep repeating this over and over.
    }


    GetMinMax(busV)
    GetMinMax(gTemp)

    if((buffer->wodHKPayload.common.moduTemp & 0xf0) == 0){ // Get minmax only if it is not stale or invalid
        GetMinMax(moduTemp);
    }
    GetMinMax(Xspin)
    GetMinMax(Yspin)
    GetMinMax(Zspin)
    GetMinMax(Xaccel)
    GetMinMax(Yaccel)
    GetMinMax(Zaccel)
    GetMinMax(Xmag)
    GetMinMax(Ymag)
    GetMinMax(Zmag)
    GetMinMax(rssi);
    GetMinMax(ICR3VProt)
    GetMinMax(ICRTemp)
    GetMinMax(TxPAi)
    GetMinMax(TxTemp)
    GetMinMax(ReflectedPwr) /* This might be 0 because it was collected with the Tx was off */
    GetMinMax(FwdPower)  /* This might be 0 because it was collected with the Tx was off */

    //todo: Make sure these are 0 when 10GHz tx is off
    GetMinMax(Power10GHz)
    GetMinMax(Current10GHz);
    //todo: Power10GHz and Current 10GHz need to be added to fetch from PSOC
    GetMinMax(ZynqTemp)
    GetMinMax(SDRTcvrTemp)
    GetMinMax(SDRXspin)
    GetMinMax(SDRYspin)
    GetMinMax(SDRZspin)
    GetMinMax(SDRXmag)
    GetMinMax(SDRYmag)
    GetMinMax(SDRZmag)
    GetMinMax(SDRXaccel)
    GetMinMax(SDRZaccel)
    GetMinMax(CIU3V3_1)
    GetMinMax(CIU3V3_2)
    GetMinMax(CIUVsys)
    GetMinMax(CIU12v)
    GetMinMax(deorbitTemp)
    GetMinMax0OK(MinusYVolts)
    GetMinMax0OK(MinusXVolts)
    GetMinMax0OK(XVolts)
    GetMinMax0OK(YVolts)
    GetMinMax(MinusYTemp)
    GetMinMax(MinusXTemp)
    GetMinMax(XTemp)
    GetMinMax(YTemp);
    if(buffer->wodHKPayload.common.DCT1RSSI != 0xFF){ // This means the RSSI is not valid
        GetMinMax(DCT1RSSI);
    }
    if(buffer->wodHKPayload.common.DCT2RSSI != 0xFF){ // This means the RSSI is not valid
        GetMinMax(DCT2RSSI);
    }
    if(buffer->wodHKPayload.common.RT1Temp != 0xFF){ // This means we have not gotten a good temp yet
        GetMinMax(RT1Temp)
    }
    if(buffer->wodHKPayload.common.RT2Temp != 0xFF){
        GetMinMax(RT2Temp)
    }
    if(buffer->wodHKPayload.common.DCT1Power != 0xFF){
        GetMinMax(DCT1Power)
    }
    if(buffer->wodHKPayload.common.DCT2Power != 0xFF){
        GetMinMax(DCT2Power)
    }
    GetMinMax(IHUcpuTemp)
    GetMinMax(CSS1)
    GetMinMax(CSS2)
    GetMinMax(CSS3)
    GetMinMax(CSS4)
    GetMinMax(CSS5)
    GetMinMax(CSS6)
    GetMinMax(CSS7)
    GetMinMax(CSS8)

    if ((updateMinCnt != 0) || updateMinCrc) { //Rewrite and recompute CRC if anything changed or the CRC was bad
        if(updateMinCnt != 0){
            MinInfo.deltaMin.IHUresetCnt = currentTime.IHUresetCnt;
            MinInfo.deltaMin.METcount = currentTime.METcount;
        }
        MinInfo.MinCrc = computeCRC((void *) &MinInfo, sizeof(MinInfo)/4-1); /* update the CRC field */
        NVstatus=writeNV(&MinInfo, sizeof(MinInfo), ExternalMRAMData, (int) &(ptr->minData));
        if (NVstatus == false) ReportError(MRAMwrite, false, ReturnAddr, (int) MinMaxUpdate);

    }
    if ((updateMaxCnt != 0) || updateMaxCrc) { //Rewrite and recompute CRC if anything changed or the CRC was bad
        if(updateMaxCnt != 0){
            MaxInfo.deltaMax.IHUresetCnt = currentTime.IHUresetCnt;
            MaxInfo.deltaMax.METcount = currentTime.METcount;
        }
        MaxInfo.MaxCrc = computeCRC((void *) &MaxInfo, sizeof(MaxInfo)/4-1); /* update the CRC field */
        NVstatus=writeNV(&MaxInfo, sizeof(MaxInfo), ExternalMRAMData, (int) &(ptr->maxData));
        if (NVstatus == false) ReportError(MRAMwrite, false, ReturnAddr, (int) MinMaxUpdate);
    }
}

