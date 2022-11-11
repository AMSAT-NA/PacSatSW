/*
 * Burns Fisher, AMSAT-NA Fox-1 and Golf
 */



/* Fox header files */

#include <pacsat.h>
#include "TMS570Hardware.h"
#include "ihuSpecificIncludes.h"
#include "I2cAddresses.h"
#include "nonvolManagement.h"
#include "TelemetryCollection.h"
#include "nonvol.h"
#include "gpioDriver.h"
#include "watchdogSupport.h"
#include "MET.h"
#include "downlink.h"
#include "CANSupport.h"
#include "CANQueueManagement.h"
#include "i2cDriver.h"
#include "MRAMmap.h"
#include "ADS7828.h"
#include "Max31725Temp.h"
#include "crc32.h"
#include "stdio.h"
#include "inet.h"
#include "CommandTask.h"
#include "TelemetryRadio.h"
#include "LinearInterp.h"
#include "adc.h"
#include "DownlinkControl.h"
#include "ax5043.h"
#include "MinMaxCalcs.h"
extern uint8_t SWCmdRing[SW_CMD_RING_SIZE];

/*******************************************************************************/

/**
 * @brief main entry point.
 * @par Parameters None
 * @retval void None
 * @par Required preconditions: None
 */


/*
 * Module-wide data available only within the module)
 */


/*
 * Forward Routines
 */


static void getAccessibleData(WODHkMRAM_t *buffer);
static void getMyLocalData(WODHkMRAM_t *buffer);

static void WriteHkWOD(WODHkMRAM_t *buffer, bool getDataFromCAN);

uint16_t uint16_swap(uint16_t x);

//Extern definitions
uint16_t WODFrequency,NumWODSaved;
int32_t WODHkStoreIndex,WODHkFetchIndex;
int32_t WODSciStoreIndex,WODSciFetchIndex;
int32_t WODRagStoreIndex,WODRagFetchIndex;

extern bool InSafeMode;
extern rt1Errors_t localErrorCollection;

static bool MinMaxCRCError;
static CanID1 CANDiagID1;

int telemetryIndex = 0;
WODHkMRAM_t telemetrySet[2];
WODHkMRAM_t *RealTimeTelemetry = &telemetrySet[0]; // RealTimeTelemetry is what the "GetRealtimeTelemetry" routine uses

portTASK_FUNCTION(TelemetryCollectTask, pvParameters )
{ 
    /*
     * This task is designed to collect telemetry information from around the spacecraft and to
     * calculate the min/max values for those where it makes sense.  In addition, it communicates
     * telemetry between the current RT-IHU processor and the other processor and to the LIHU.
     */
    WODHkMRAM_t *dataSetPtr;

    /*
     * First get information that is stored in non-volatile memory
     */

    WODFrequency = ReadMRAMWODFreq();
    //todo: Change this if commanded to change
    NumWODSaved = ReadMRAMWODSaved();
    WODHkStoreIndex = ReadMRAMWODHkStoreIndex();
    WODSciStoreIndex = ReadMRAMWODSciStoreIndex();
    WODRagStoreIndex = ReadMRAMWODRagStoreIndex();
    WODHkFetchIndex = ReadMRAMWODHkDownlinkIndex();
    WODSciFetchIndex = ReadMRAMWODSciDownlinkIndex();
    WODRagFetchIndex = ReadMRAMWODRagDownlinkIndex();
    MinMaxCRCError = ReadMRAMBoolState(StateMinMaxCRCError);

    vTaskSetApplicationTaskTag((xTaskHandle) 0, (pdTASK_HOOK_CODE)TelemetryWD); // For watchdog when it is called with "current task"
    ReportToWatchdog(TelemetryWD); /* Make sure we don't time out */
    InitInterTask(ToTelemetry,4);
    localErrorCollection.valid = 1;
    CANDiagID1 = CANId1InterCPU_RT1Diag;


    /*
     * Don't start collecting until everything else is started *
     * We don't want to get un-initialized data and apply it  *
     * to min/max!
     */
    vTaskDelay(WATCHDOG_SHORT_WAIT_TIME);
    ReportToWatchdog(TelemetryWD); /* Tell the WD we are ok after that delay */
    I2CDevicePoll(); /* verify that ICR, CIU, etc are communicating over I2c */
    ReportToWatchdog(TelemetryWD); /* Tell the WD we are ok after that delay */
    METTelemetryReady(); // Tell MET to send a message every few seconds


    /*
     * Here is the main loop of the program.  We run it every 4 seconds (approx) when the
     * MET module or someone else sends a message.  What happens then depends a lot on whether
     * this CPU is in control, and if not whether it can talk to the bus at all.
     */
    while (true) {
        Intertask_Message msg;
        bool status;
        ReportToWatchdog(TelemetryWD); // Make sure it knows we are still running!
        status = WaitInterTask(ToTelemetry, WATCHDOG_SHORT_WAIT_TIME, &msg);
        ReportToWatchdog(TelemetryWD); // Make sure it knows we are still running!
        if (status == pdFAIL){
            ReportError(RTOSfailure, false, ReturnAddr,(int) TelemetryCollectTask);
        }
        if((msg.MsgType == TelemSendErrorsResetMsg) || (msg.MsgType == TelemSendErrorsPowerCycleMsg)){
            /*
             * Here we got a message from the error module saying that we have a fatal error reported and
             * we need to send it to the other CPU so it won't get forgotten.  We should already have switched
             * to the other CPU. but hopefully this one can run long enough with that error to get the data
             * over.
             */
            CANSendLongMessage(CAN2,CANDiagID1,Telemetry,0,(uint8_t *)&localErrorCollection,
                               (uint32_t)sizeof(rt1Errors_t));
            if(msg.MsgType == TelemSendErrorsResetMsg){
                ProcessorReset();
            } else {
                ForceExternalWatchdogTrigger(); // This is how we power cycle
            }

        } else { //MsgType == TelemCollect
            /*
             * Here we get a buffer to put telemetry in (we double buffer it so we are not
             * filling while the downlink is fetching)
             */

            dataSetPtr = &telemetrySet[telemetryIndex];
            getMyLocalData(dataSetPtr); // Always get data that only I can read
            {
                /* I am sending telemetry.  Get it from everyone */
                /*
                 *
                 */
                getAccessibleData(dataSetPtr); /* Get all the local stuff */
                 /*
                 * Last bus voltage is what we use to determine if we are going into auto-safe mode (see
                 * control task).  Normally we use the CIU data, but if that becomes invalid, then use
                 * LIHU data.
                 */

            }
            taskYIELD();
            /*
             * Point to the latest data collected to use when the downlink control calls for
             * a realtime downlink buffer.  We will also use this for the WOD data if it is time.
             */
            RealTimeTelemetry = dataSetPtr; // Remember the data buffer we are working on.
            //telemetryIndex = nextBuffer(telemetryIndex); // Get ready to use the next one
            ReportToWatchdog(TelemetryWD);
            {
                /*
                 * The rest of the stuff does not happen until things have settled a bit after booting.
                 * That includes whole orbit data collection and min/max processing.
                 */
                if(IsStabilizedAfterBoot()){
                    /*
                     * All this stuff stores timestamps, so we want to be sure we have a valid time, and also that
                     * we have not started to do it too soon.
                     */
                    MinMaxUpdate(RealTimeTelemetry);
                    ReportToWatchdog(TelemetryWD);
                    /*
                     * Do Whole Orbit Data
                     * WODFrequencyCounter is set to the number of data collection intervals between WOD
                     * storage.  WODFrequency can be commanded to a different value potentially. And if
                     * WODStoreIndex is NO_BUFFER_AVAILABLE, we are not going to use it or increment it.  This keeps us
                     * from collecting data immediately after preflight init.
                     */
#if 0
                    if(--WODFrequencyCounter < 0 && WODHkStoreIndex >=0){
                        //WODSciMRAM_t wodSci;
                        logicalTime_t time;
                        WODFrequencyCounter=WODFrequency;
                        WriteHkWOD(RealTimeTelemetry,false);
                        ReportToWatchdog(TelemetryWD);

                    }
#endif
                }
            }
        } /* End of Collect Message */
    } /* End of while loop */
} /* end of task */

/*
 * *****************************************************************************************
 * Here are the places where we actually load the telemetry.  It comes from different places
 * depending on which processor is in control
 * *****************************************************************************************
 */

void getAccessibleData(WODHkMRAM_t *buffer){
    /*
     * This routine should be called only on the processor that owns the bus and is sending the
     * telemetry.  It gets most of the telemetry data either directly from the bus or from
     * local sources that it won't be sending anywhere else.
     */
    // Here we get it from the bus all ourselves
    logicalTime_t time;
    uint8_t temp;
    getTimestamp(&time);
    /*
     * This routine is only called if we are in control and own the bus, so we can certainly
     * get the temp.  (We got it before only if we did NOT have the bus.  If we have the bus but
     * are not in control, then the LIHU gets it)
     */
    Get8BitTemp31725(&temp);
    {
        buffer->wodHKPayload.common.RT1Temp = temp;
    }
    buffer->wodHKPayload.wodInfo.WODTimestampReset = time.IHUresetCnt;
    buffer->wodHKPayload.wodInfo.WODTimestampUptime = time.METcount;
    buffer->wodHKPayload.common2.halfDupMode = !StandbyReceiverOk();
#if 0
    getCSSData(buffer);
    getICRData(buffer);
    getSolarData(buffer);
    ReportToWatchdog(TelemetryWD);
    getErrors(buffer);
    ReportToWatchdog(TelemetryWD);
    getModeData(buffer);
    getCounters(buffer);
#endif
}

/* Fetch and set the MRAM error flag -- note: it is cleared only by clearing the Min/max values*/

bool GetMramCRCState(void){
    return MinMaxCRCError;
}
void SetMramCRCError(void){
    MinMaxCRCError = true;
    WriteMRAMBoolState(StateMinMaxCRCError,MinMaxCRCError);
}
void SetMramCRCGood(void){
    MinMaxCRCError = false;
    WriteMRAMBoolState(StateMinMaxCRCError,MinMaxCRCError);
}
/*
 * Here is the code to deal with Whole Orbit Data.  It understand both in control (in which case it stores
 * the current telemetry and also send it to the other processors) or not in control (in which case it stores
 * WOD received from the in-control processor (possibly by way of the active RTIHU)
 */

void WriteHkWOD(WODHkMRAM_t *latestData, bool getDataFromCAN){
    /*
     * Collect and write into MRAM a buffer of housekeeping data.  The data
     * comes from different places as explained below:
     *
     * 'getDataFromCAN says that the majority of the data will come from another CPU
     * and we get it over the CAN bus.  (We don't pay attention to where it came from
     * the LIHU or the active RT-IHU)  In this case, we ignore "latestData" coming from
     * the caller.  Our job here is just to keep more-or-less up-to-date with the controlling
     * CPU, so we write the WOD data, but also use it to calculate min/max
     *
     * If 'getDataFromCAN' is false, then use the data the is coming in as "latestData' to
     * write into WOD.  Min/max has already been done.
     */


    WODHkMRAM_t MRAMdata;
    MRAMmap_t *MRAMAddresses=0;
    bool NVstat;
    if(WODHkStoreIndex < 0)return; // Just after preflight init; not saving WOD

    /*
     * LatestData has the (duh) latest data whether it be from our caller or from another
     * CPU via CAN.  Put it into the MRAM data structure and write it out WOD.
     */
    //loadCommonFields(&MRAMdata.wodHKPayload.common,latestData);
    //loadCommonFields2(&MRAMdata.wodHKPayload.common2,latestData);
    MRAMdata.wodHKPayload.wodInfo.WODTimestampReset = ntohs(latestData->wodHKPayload.wodInfo.WODTimestampReset);
    MRAMdata.wodHKPayload.wodInfo.WODTimestampUptime = ntohl(latestData->wodHKPayload.wodInfo.WODTimestampUptime);
    //
    MRAMdata.Crc = computeCRC((uint32_t *) &MRAMdata, CRClen(MRAMdata));
    NVstat = writeNV(&MRAMdata, sizeof(MRAMdata), ExternalMRAMData,
                     (int) &(MRAMAddresses->WODHousekeeping[WODHkStoreIndex]));
    if (NVstat == false) { ReportError(MRAMwrite, false, ReturnAddr, (int) WriteHkWOD); }
    if(++WODHkStoreIndex >= NumWODSaved)WODHkStoreIndex=0;
    WriteMRAMWODHkStoreIndex(WODHkStoreIndex);

}

void getMyLocalData(WODHkMRAM_t *buffer){
    uint8_t temp;
    bool txIsOn;
    txIsOn = GPIOIsOn(PAPower);
    if(txIsOn) {
        adcStartConversion(adcREG1,adcGROUP1); // Start measuring the PA Power etc
        Get8BitTemp31725(&temp);
    }
    {
        buffer->wodHKPayload.common.RT1Temp = temp;
        buffer->wodHKPayload.common2.PrimMRAMstatus = ReadMRAMStatus();
        buffer->wodHKPayload.common2.DCT1PwrCnt = GetDCTPowerFlagCnt();
        buffer->wodHKPayload.common2.PA1PwrCnt = GetPAPowerFlagCnt();
        buffer->wodHKPayload.common2.RTIHU1Resets = (uint8_t)(0xff & getResets());
        {
            bool halfDup = !StandbyReceiverOk();
            buffer->wodHKPayload.common2.halfDupMode = halfDup;
            if(halfDup){
                // If we are in half duplex, only update the RSSI if the receiver is actually on
                if(IsRxing()){
                    buffer->wodHKPayload.common.DCT1RSSI = get_rssi();
                }
            } else {
                // We are not in half duplex so our receiver is not on.  Hence we say that our RSSI is
                // ff.
                buffer->wodHKPayload.common.DCT1RSSI = 0xff;
            }
        }
    }
    //todo: Get DCT status too
#if 0
    getCounters(buffer);
    if(txIsOn){
        while(adcIsConversionComplete(adcREG1,adcGROUP1)==0){
            vTaskDelay(1);
        }
        adcGetData(adcREG1,adcGROUP1,adcData);
        adcStopConversion(adcREG1,adcGROUP1); // Start measuring the PA Power etc
        power = (uint8_t)(adcData[0].value & 0xff);
    } else {
        power = 0xff; // Tx is not on.  Don't measure power
    }
    if(primary){
        buffer->wodHKPayload.common.DCT1Power = power;
    } else {
        buffer->wodHKPayload.common.DCT2Power = power;
    }
#endif
}


/* this function converts between big-endian and little-endian storage formats */
uint16_t uint16_swap(uint16_t x) {
    return((x<<8) | (x>>8));
}
uint32_t computeCRC(uint32_t *address, int words){
    /*
     * The 'computeCRC' name comes from the Fox software and there it uses the CRC hardware in the STM32.
     * The hardware in the current hardware seems to work on 64 bits, so I'm just calling a CRC routine
     * that I fetched from the web somewhere called crc32.  Hopefully this can all get resolved sooner
     * or later.
     */
    return crc32(address,words);
}

void ChangeWODSaved(uint16_t numSaved){
    /*
     * Command to change the number of WOD data items that we save.  It changes both science
     * and housekeeping and if the size is reduced, it abandons the values that are in the
     * part of memory that becomes unused, although if we are sending something from that area
     * it will finish sending all the WODs in the discontinued area.
     */
    if(numSaved > MAX_WOD_SAVED)numSaved=MAX_WOD_SAVED;
    // If we are currently writing WOD into the area of memory that we are going to stop
    // using, change the index
    //if(WODSciStoreIndex > numSaved)WODSciStoreIndex = 0; // Science store location is in the removed area
    if(WODHkStoreIndex > numSaved)WODHkStoreIndex = 0; //Housekeeping store location is in the removed area

    NumWODSaved = numSaved;
    WriteMRAMWODSaved(numSaved);
}
void getErrors(WODHkMRAM_t *buffer){
    buffer->wodHKPayload.common2.I2CfailureICR = !ICRTelemIsOk();
    buffer->wodHKPayload.common2.I2CfailureSolarV = !SolarTelemIsOk();
    buffer->wodHKPayload.common2.I2CfailureSunSense = !CSSTelemIsOk();
}

void getCounters(WODHkMRAM_t *buffer){
    int i;
    buffer->wodHKPayload.common2.TLMresets = getMinMaxResetCount();
    buffer->wodHKPayload.common2.wodSize = (NumWODSaved/4);
    for(i=0;i<SW_CMD_RING_SIZE;i++){
        buffer->wodHKPayload.common2.swCmds[i] = SWCmdRing[i];
    }
    buffer->wodHKPayload.common2.swCmdCntDCT = GetSWCmdCount();
}


void getModeData(WODHkMRAM_t *buffer) {
    buffer->wodHKPayload.common2.AutoSafeAllowed = IsAutoSafeAllowed();
    buffer->wodHKPayload.common2.AutoSafeModeActive =InAutoSafeMode();
    buffer->wodHKPayload.common2.transponderEnabled = IsTransponderEnabled();
    //buffer->wodHKPayload.common2.inEclipse = WeAreInEclipse();
    buffer->wodHKPayload.common2.vucDisabled = GetVUCDisabled();
}

#define Change12to8bits(bit16) ((uint8_t)(((bit16+8)&0xfff)>>4))
//In the above, we want to removed the least significant 4 bits.  First add 0b1000 to round up.
//Then then AND off any high bits and shift the bottom 4 bits off the bottom.

#if 0
static bool getICRData(WODHkMRAM_t *buffer) {
    /*
     * First get the ICR data
     */

    if (ICRTelemIsOk()  == true) { /* If the 7828 is not responding, exit */
        ICRinfo_t icr;
        static uint8_t rssi=0;

        // All ICR A/D Converter values are scaled by .69.  That is measuring 3.3V goes into the
        // ADC as 3.3*.69 = 2.29.  Another way to think of it:  An ADC count of 4095 = 2.5V, which
        // means the measured voltage is 3.62V.  If you only look at 8 bits, then 255=3.62V and each
        // count is 3.62/255 or .014V

        getADCchannels(sizeof(ICRinfo_t)/sizeof(uint16_t), ICR_ADC_I2C_PORT, ICR_ADC_I2C_ADDRESS,
                       (uint16_t *) &(icr));
        if(icr.RxPowerOn > 2048){
            buffer->wodHKPayload.common2.ICRRxOn = true;  // This is really just high and low
            rssi = Change12to8bits(icr.RSSI);
        }
        buffer->wodHKPayload.common.rssi = rssi; // Only change when the RSSI is valid
        buffer->wodHKPayload.common.TxTemp = (LinearInterpolate(icr.TxTemp,1780,315)-600);  //TMP36 temp formula to get tenths of a degree
        buffer->wodHKPayload.common.ICRTemp = (LinearInterpolate(icr.TxTemp,1780,315)-600);  //TMP36 temp formula to get tenths of a degree
        buffer->wodHKPayload.common.ICR3VProt =Change12to8bits(icr.RegulatedProt3V);
        buffer->wodHKPayload.common.LtVGACtl =Change12to8bits(icr.VGA);
        //todo: This needs to be averaged

        //if(TransmitterIsOnAndStable()){ //Comes from downlink control
        /* We will only collect the power when the transmitter is turned on */
        buffer->wodHKPayload.common.ReflectedPwr = Change12to8bits(icr.ReflectedPower);
        buffer->wodHKPayload.common.FwdPower = Change12to8bits(icr.ForwardPower);
    } else {
        buffer->wodHKPayload.common.ICRTemp =0;//Regulated2dot5V);
        buffer->wodHKPayload.common.ICR3VProt =0;//RegulatedProt3V);
        buffer->wodHKPayload.common.rssi =0;//RSSI);
        buffer->wodHKPayload.common.LtVGACtl =0;//VGA);
        buffer->wodHKPayload.common.TxTemp = 0;
        buffer->wodHKPayload.common.TxPAi =0;//TxCurrent);
        buffer->wodHKPayload.common.ReflectedPwr = 0;//ReflectedPower);
        buffer->wodHKPayload.common.FwdPower = 0;//ForwardPower);

        return false;
    }
    return true;
}

static bool getSolarData(WODHkMRAM_t *buffer){
    if(SolarTelemIsOk()){
        SolInfo_t solar;
        uint8_t v1,v2,v3,v4;
        getADCchannels(sizeof(SolInfo_t)/sizeof(uint16_t), SOLAR_ADC_I2C_PORT, SOLAR_ADC_I2C_ADDRESS,
                       (uint16_t *) &(solar));
        v1 = buffer->wodHKPayload.common.XVolts = Change12to8bits(solar.XVolts);
        v2 = buffer->wodHKPayload.common.YVolts = Change12to8bits(solar.YVolts);
        v3 = buffer->wodHKPayload.common.MinusXVolts = Change12to8bits(solar.MinusXVolts);
        v4 = buffer->wodHKPayload.common.MinusYVolts = Change12to8bits(solar.MinusYVolts);
        buffer->wodHKPayload.common.XTemp = Change12to8bits(solar.XTemp);
        buffer->wodHKPayload.common.YTemp = Change12to8bits(solar.YTemp);
        buffer->wodHKPayload.common.MinusXTemp = Change12to8bits(solar.MinusXTemp);
        buffer->wodHKPayload.common.MinusYTemp = Change12to8bits(solar.MinusYTemp);
        InEclipse = ((v1<SOLAR_VOLTS_IN_SUN_MIN) && (v2<SOLAR_VOLTS_IN_SUN_MIN) &&
                 (v3<SOLAR_VOLTS_IN_SUN_MIN) && (v4<SOLAR_VOLTS_IN_SUN_MIN));
    } else {
        buffer->wodHKPayload.common.XVolts = 0;//XVolts);
        buffer->wodHKPayload.common.YVolts = 0;//YVolts);
        buffer->wodHKPayload.common.MinusXVolts = 0;//MinusXVolts);
        buffer->wodHKPayload.common.MinusYVolts = 0;//MinusYVolts);
        buffer->wodHKPayload.common.XTemp = 0;//XTemp);
        buffer->wodHKPayload.common.YTemp = 0;//YTemp);
        buffer->wodHKPayload.common.MinusXTemp = 0;//MinusXTemp);
        buffer->wodHKPayload.common.MinusYTemp = 0;//MinusYTemp);
        return false;
    }
    return true;
}

static bool getCSSData(WODHkMRAM_t *buffer){
    if(CSSTelemIsOk()){
        CSSInfo_t sunSense;
        getADCchannels(sizeof(CSSInfo_t)/sizeof(uint16_t), CSS_ADC_I2C_PORT, CSS_ADC_I2C_ADDRESS,
                       (uint16_t *) &(sunSense));
        buffer->wodHKPayload.common.CSS1 = Change12to8bits(sunSense.CSS1);
        buffer->wodHKPayload.common.CSS2 = Change12to8bits(sunSense.CSS2);
        buffer->wodHKPayload.common.CSS3 = Change12to8bits(sunSense.CSS3);
        buffer->wodHKPayload.common.CSS4 = Change12to8bits(sunSense.CSS4);
        buffer->wodHKPayload.common.CSS5 = Change12to8bits(sunSense.CSS5);
        buffer->wodHKPayload.common.CSS6 = Change12to8bits(sunSense.CSS6);
        buffer->wodHKPayload.common.CSS7 = Change12to8bits(sunSense.CSS7);
        buffer->wodHKPayload.common.CSS8 = Change12to8bits(sunSense.CSS8);
    } else {
        buffer->wodHKPayload.common.CSS1 = 0;//1);
        buffer->wodHKPayload.common.CSS2 = 0;//2);
        buffer->wodHKPayload.common.CSS3 = 0;//3);
        buffer->wodHKPayload.common.CSS4 = 0;//4);
        buffer->wodHKPayload.common.CSS5 = 0;//5);
        buffer->wodHKPayload.common.CSS6 = 0;//6);
        buffer->wodHKPayload.common.CSS7 = 0;//7);
        buffer->wodHKPayload.common.CSS8 = 0;//8);
        return false;
    }
    return true;
}

bool WeAreInEclipse(void){
    if(ThisIHUInControl() && SolarTelemIsOk()){
        VirtualInEclipse = InEclipse;
    }
    return VirtualInEclipse;
}
void SetInEclipse(bool eclipse){
    VirtualInEclipse = eclipse;
}
#endif
/* This function returns the current voltage level on the power bus. */
bool getBusVoltage(uint8_t *volts) {
#if 0
    static bool inControlLastTime=false; //
    if(ThisIHUInControl() && (lastBusVoltage != 255)){
        if(!inControlLastTime){
            // Control has changed.  Wait for another update.
            inControlLastTime = true;
            lastBusVoltage = INVALID_CAN_BYTE;
            return false;
        }
        *volts = lastBusVoltage;
        return true;
    } else return false;
#endif
    return false;
}

