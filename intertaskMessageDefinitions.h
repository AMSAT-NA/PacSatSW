/*
 * IntertaskMessagesDefinitions.h
 *
 *  Created on: Mar 13, 2020
 *      Author: bfisher
 */

#ifndef INTERTASKMESSAGEDEFINITIONS_H_
#define INTERTASKMESSAGEDEFINITIONS_H_


/* Messages for the downlinkControl*/
typedef enum {
    /* These messages are for the downlink task.
     * The explicit values are required since they index a table
     */
    DwnlnkIdleTimeout=1,DwnlnkBeaconTimeout,DwnlnkTelemetryFrameComplete,
    DwnlnkEnterSafeMode,DwnlnkEnterHealthMode,DwnlnkEnterAutoSafeMode,DwnlnkInhibitTransmit,
    DwnlnkEnableTransmit,
    DwnlnkEnterScienceMode,
    DwnlnkEnterEclipseSafeMode,
    DWNLNK_NUMBER_OF_STATE_MESSAGES,
    //End of table indices
    DwnlnkCollectTelemetry,DwnlnkProgramModulator,DwnlnkControlChange,
    /*
     * These are for the command task
     */
    CmdTypeHardware = 0, /* We might not care if this starts with 0 */
    CmdTypeRawSoftware,
    CmdTypeValidatedSoftwareLocal,
    CmdTypeValidatedSoftwareCAN,
    CmdTypeRawSoftwareCAN,
    CmdControlHasChanged,
    /*
     * These are for the CAN task
     */
    CANMessageReceived,
//    /*
//     * These are for RIHU Coordination
//     */
//    CoordLIHU0IntMsg,
//    CoordLIHU1IntMsg,
//    CoordRxCANMsg,
//    CoordSendStatusMsg,
//    CoordSendPreflightInit,
//    CoordSendReset,
//    CoordSendInOrbit,
//    CoordForceStateCheckMsg,

    /*
     * For messages to the radio task
     */
    DCTInterruptMsg,
    DCTPowerFlagMsg,
    PAPowerFlagMsg,
    DCTInitMessage,
    /*
     * Messages to the telemetry and Control task
     */
    TelemSendPbStatus,
    TelemSendUplinkStatus,
    TelemCollectMsg,
    TelemSendErrorsResetMsg,
    TelemSendErrorsPowerCycleMsg

}IntertaskMessageType;


#endif /* INTERTASKMESSAGEDEFINITIONS_H_ */
