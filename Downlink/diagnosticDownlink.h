/*
 * diagnosticDownlink.h
 *
 *  Created on: Jun 24, 2023
 *      Author: g0kla
 */

#ifndef DOWNLINK_DIAGNOSTICDOWNLINK_H_
#define DOWNLINK_DIAGNOSTICDOWNLINK_H_

typedef struct __attribute__((__packed__)) {
    unsigned int wdReports : 9;
    unsigned int errorCode : 5;
    unsigned int taskNumber : 4;
    unsigned int previousTask : 4;
    unsigned int earlyResetCount : 3;
    unsigned int wasStillEarlyInBoot : 1;
    unsigned int valid : 1;
    unsigned int pad319 : 5;
    unsigned int errorData : 32;
    unsigned int RAMCorAddr1 : 32;
    unsigned int RAMCorAddr2 : 32;
    unsigned int ROMCorAddr1 : 32;
    unsigned int ROMCorAddr2 : 32;
    unsigned int DCTCmdFailCRCCnt : 8;
    unsigned int DCTCmdFailTimeCnt : 8;
    unsigned int DCTCmdFailAuthenticateCnt : 8;
    unsigned int DCTCmdFailCommandCnt : 8;
    unsigned int DCTCmdFailNamespaceCnt : 8;
    unsigned int I2C1ErrorCnt : 8;
    unsigned int I2C2ErrorCnt : 8;
    unsigned int I2C1ResetCnt : 8;
    unsigned int I2C2ResetCnt : 8;
    unsigned int I2C1RetryCnt : 8;
    unsigned int I2C2RetryCnt : 8;
    unsigned int SPIRetryCnt : 8;
    unsigned int MramCRCCnt : 8;
    unsigned int MramRdErrorCnt : 8;
    unsigned int MramWtErrorCnt : 8;
    unsigned int FlashCorCnt : 8;
    unsigned int RamEvenCorCnt : 8;
    unsigned int RamOddCorCnt : 8;
    unsigned int PLLSlipCnt : 8;
    unsigned int ClockMonitorFailCnt : 8;
    unsigned int VIMRamParityCnt : 8;
    unsigned int CAN1RamParityCnt : 8;
    unsigned int CAN2RamParityCnt : 8;
    unsigned int DCC1ErrorCnt : 8;
    unsigned int DCC2ErrorCnt : 8;
    unsigned int N2HET2RamParityCnt : 8;
    unsigned int IOMMAccvioCnt : 8;
    unsigned int nonFatalCnt : 8;
    unsigned int TxDroppedPacketCnt : 8;
    unsigned int RxDroppedPacketCnt : 8;
    unsigned int SWVersion : 16;
    unsigned int pad362 : 16;
} diagnostictelemetry_t;

void print_diagnostictelemetry(diagnostictelemetry_t payload) {
    printf("RAW PAYLOAD: diagnostictelemetry\n");
    printf("RTIHU1: WD Tasks Ok: %d  ",payload.wdReports);
    printf("Cause of Reset: %d  ",payload.errorCode);
    printf("Task executing at reset: %d  ",payload.taskNumber);
    printf("Previous task before reset: %d  ",payload.previousTask);
    printf("\n");
    printf("Resets in a row that have been too soon: %d  ",payload.earlyResetCount);
    printf("Was this reset still early in boot: %d  ",payload.wasStillEarlyInBoot);
    printf("RTIHU1 Data Valid: %d  ",payload.valid);
    printf("pad to 32 bits: %d  ",payload.pad319);
    printf("\n");
    printf("Error Data: %d  ",payload.errorData);
    printf("Primary SRAM Corrected Addr 1: %d  ",payload.RAMCorAddr1);
    printf("Primary SRAM Corrected Addr 2: %d  ",payload.RAMCorAddr2);
    printf("Primary Flash Corrected Addr 1: %d  ",payload.ROMCorAddr1);
    printf("\n");
    printf("Primary Flash Corrected Addr 2: %d  ",payload.ROMCorAddr2);
    printf("DCT1CmdFailCRC: %d  ",payload.DCTCmdFailCRCCnt);
    printf("DCT1CmdFailTime: %d  ",payload.DCTCmdFailTimeCnt);
    printf("DCT1CmdFailAuthenticate: %d  ",payload.DCTCmdFailAuthenticateCnt);
    printf("\n");
    printf("DCT1CmdFailCommand: %d  ",payload.DCTCmdFailCommandCnt);
    printf("DCT1CmdFailNamespace: %d  ",payload.DCTCmdFailNamespaceCnt);
    printf("RT1I2C1Error: %d  ",payload.I2C1ErrorCnt);
    printf("RT1I2C2Error: %d  ",payload.I2C2ErrorCnt);
    printf("\n");
    printf("RT1I2C1Reset: %d  ",payload.I2C1ResetCnt);
    printf("RT1I2C2Reset: %d  ",payload.I2C2ResetCnt);
    printf("RT1 I2c1 Retry Count: %d  ",payload.I2C1RetryCnt);
    printf("RT1 I2c2 Retry Count: %d  ",payload.I2C2RetryCnt);
    printf("\n");
    printf("RT1SPIRetries: %d  ",payload.SPIRetryCnt);
    printf("RT1 MRAM CRC Fail count: %d  ",payload.MramCRCCnt);
    printf("RT1 MRAM Read Error Count: %d  ",payload.MramRdErrorCnt);
    printf("RT1 MRAM Write Error Count: %d  ",payload.MramWtErrorCnt);
    printf("\n");
    printf("RT1FlashCorrectable Count: %d  ",payload.FlashCorCnt);
    printf("RT1 Ram Even Bank Correctable Count: %d  ",payload.RamEvenCorCnt);
    printf("RT1 Ram Odd Bank Correctable Count: %d  ",payload.RamOddCorCnt);
    printf("RT1PLLSlip: %d  ",payload.PLLSlipCnt);
    printf("\n");
    printf("RT1ClockMonitorFail: %d  ",payload.ClockMonitorFailCnt);
    printf("RT1VIMRamParity: %d  ",payload.VIMRamParityCnt);
    printf("RT1CAN1RamParity: %d  ",payload.CAN1RamParityCnt);
    printf("RT1CAN2RamParity: %d  ",payload.CAN2RamParityCnt);
    printf("\n");
    printf("RT1DCC1Error: %d  ",payload.DCC1ErrorCnt);
    printf("RT1DCC2Error: %d  ",payload.DCC2ErrorCnt);
    printf("RT1N2HET2RamParity: %d  ",payload.N2HET2RamParityCnt);
    printf("RT1IOMMAccvio: %d  ",payload.IOMMAccvioCnt);
    printf("\n");
    printf("Non-fatal error count: %d  ",payload.nonFatalCnt);
    printf("Non-fatal error count: %d  ",payload.TxDroppedPacketCnt);
    printf("Non-fatal error count: %d  ",payload.RxDroppedPacketCnt);
    printf("RTIHU1 Version Number: %d  ",payload.SWVersion);
    printf("\n");
    printf("Fill to 32-bits: %d  ",payload.pad362);
    printf("\n");
}





#endif /* DOWNLINK_DIAGNOSTICDOWNLINK_H_ */
