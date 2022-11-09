#ifndef __HET_EMU_I2C_h
#define __HET_EMU_I2C_h

#define HET_v2 1
#define AID1_7

#include "std_nhet.h"

#define HET_Master_Start_1	(e_HETPROGRAM1_UN.Program1_ST.Master_Start_1)
#define pHET_Master_Start_1  	0

#define HET_Check_Stat0_1	(e_HETPROGRAM1_UN.Program1_ST.Check_Stat0_1)
#define pHET_Check_Stat0_1  	1

#define HET_State0_1	(e_HETPROGRAM1_UN.Program1_ST.State0_1)
#define pHET_State0_1  	2

#define HET_Clear_Time2_1	(e_HETPROGRAM1_UN.Program1_ST.Clear_Time2_1)
#define pHET_Clear_Time2_1  	3

#define HET_Clear_Time6_1	(e_HETPROGRAM1_UN.Program1_ST.Clear_Time6_1)
#define pHET_Clear_Time6_1  	4

#define HET_Clear_Time9_1	(e_HETPROGRAM1_UN.Program1_ST.Clear_Time9_1)
#define pHET_Clear_Time9_1  	5

#define HET_NewDataFlag_1	(e_HETPROGRAM1_UN.Program1_ST.NewDataFlag_1)
#define pHET_NewDataFlag_1  	6

#define HET_CheckNewData_1	(e_HETPROGRAM1_UN.Program1_ST.CheckNewData_1)
#define pHET_CheckNewData_1  	7

#define HET_ClearREM_1	(e_HETPROGRAM1_UN.Program1_ST.ClearREM_1)
#define pHET_ClearREM_1  	8

#define HET_CheckTranInt_1	(e_HETPROGRAM1_UN.Program1_ST.CheckTranInt_1)
#define pHET_CheckTranInt_1  	9

#define HET_GeneTranInt_1	(e_HETPROGRAM1_UN.Program1_ST.GeneTranInt_1)
#define pHET_GeneTranInt_1  	10

#define HET_SCLWave_1	(e_HETPROGRAM1_UN.Program1_ST.SCLWave_1)
#define pHET_SCLWave_1  	11

#define HET_GetData_1	(e_HETPROGRAM1_UN.Program1_ST.GetData_1)
#define pHET_GetData_1  	12

#define HET_CheckStart_1	(e_HETPROGRAM1_UN.Program1_ST.CheckStart_1)
#define pHET_CheckStart_1  	13

#define HET_StartYes_1	(e_HETPROGRAM1_UN.Program1_ST.StartYes_1)
#define pHET_StartYes_1  	14

#define HET_AddStartBit_1	(e_HETPROGRAM1_UN.Program1_ST.AddStartBit_1)
#define pHET_AddStartBit_1  	15

#define HET_RestoreRead_1	(e_HETPROGRAM1_UN.Program1_ST.RestoreRead_1)
#define pHET_RestoreRead_1  	16

#define HET_InvR_W_1	(e_HETPROGRAM1_UN.Program1_ST.InvR_W_1)
#define pHET_InvR_W_1  	17

#define HET_CpyR_W_1	(e_HETPROGRAM1_UN.Program1_ST.CpyR_W_1)
#define pHET_CpyR_W_1  	18

#define HET_CpyBytes_1	(e_HETPROGRAM1_UN.Program1_ST.CpyBytes_1)
#define pHET_CpyBytes_1  	19

#define HET_NoStartBit_1	(e_HETPROGRAM1_UN.Program1_ST.NoStartBit_1)
#define pHET_NoStartBit_1  	20

#define HET_GoStat4_1	(e_HETPROGRAM1_UN.Program1_ST.GoStat4_1)
#define pHET_GoStat4_1  	21

#define HET_Check_Stat1_1	(e_HETPROGRAM1_UN.Program1_ST.Check_Stat1_1)
#define pHET_Check_Stat1_1  	22

#define HET_Stat1BR_1	(e_HETPROGRAM1_UN.Program1_ST.Stat1BR_1)
#define pHET_Stat1BR_1  	23

#define HET_Check_Stat2_1	(e_HETPROGRAM1_UN.Program1_ST.Check_Stat2_1)
#define pHET_Check_Stat2_1  	24

#define HET_Stat2BR_1	(e_HETPROGRAM1_UN.Program1_ST.Stat2BR_1)
#define pHET_Stat2BR_1  	25

#define HET_State2_1	(e_HETPROGRAM1_UN.Program1_ST.State2_1)
#define pHET_State2_1  	26

#define HET_TimeOut2CNT_1	(e_HETPROGRAM1_UN.Program1_ST.TimeOut2CNT_1)
#define pHET_TimeOut2CNT_1  	27

#define HET_TimeOut2OCU_1	(e_HETPROGRAM1_UN.Program1_ST.TimeOut2OCU_1)
#define pHET_TimeOut2OCU_1  	28

#define HET_Check_Stat3_1	(e_HETPROGRAM1_UN.Program1_ST.Check_Stat3_1)
#define pHET_Check_Stat3_1  	29

#define HET_Stat3BR_1	(e_HETPROGRAM1_UN.Program1_ST.Stat3BR_1)
#define pHET_Stat3BR_1  	30

#define HET_Check_Stat4_1	(e_HETPROGRAM1_UN.Program1_ST.Check_Stat4_1)
#define pHET_Check_Stat4_1  	31

#define HET_Stat4BR_1	(e_HETPROGRAM1_UN.Program1_ST.Stat4BR_1)
#define pHET_Stat4BR_1  	32

#define HET_Check_Stat5_1	(e_HETPROGRAM1_UN.Program1_ST.Check_Stat5_1)
#define pHET_Check_Stat5_1  	33

#define HET_Stat5BR_1	(e_HETPROGRAM1_UN.Program1_ST.Stat5BR_1)
#define pHET_Stat5BR_1  	34

#define HET_Check_Stat6_1	(e_HETPROGRAM1_UN.Program1_ST.Check_Stat6_1)
#define pHET_Check_Stat6_1  	35

#define HET_Stat6BR_1	(e_HETPROGRAM1_UN.Program1_ST.Stat6BR_1)
#define pHET_Stat6BR_1  	36

#define HET_State6_1	(e_HETPROGRAM1_UN.Program1_ST.State6_1)
#define pHET_State6_1  	37

#define HET_TimeOut6CNT_1	(e_HETPROGRAM1_UN.Program1_ST.TimeOut6CNT_1)
#define pHET_TimeOut6CNT_1  	38

#define HET_TimeOut6OCU_1	(e_HETPROGRAM1_UN.Program1_ST.TimeOut6OCU_1)
#define pHET_TimeOut6OCU_1  	39

#define HET_ShiftBitIn_1	(e_HETPROGRAM1_UN.Program1_ST.ShiftBitIn_1)
#define pHET_ShiftBitIn_1  	40

#define HET_BitCNT_1	(e_HETPROGRAM1_UN.Program1_ST.BitCNT_1)
#define pHET_BitCNT_1  	41

#define HET_ByteReady_1	(e_HETPROGRAM1_UN.Program1_ST.ByteReady_1)
#define pHET_ByteReady_1  	42

#define HET_NextData_1	(e_HETPROGRAM1_UN.Program1_ST.NextData_1)
#define pHET_NextData_1  	43

#define HET_CleanRecDat_1	(e_HETPROGRAM1_UN.Program1_ST.CleanRecDat_1)
#define pHET_CleanRecDat_1  	44

#define HET_Isaddress_1	(e_HETPROGRAM1_UN.Program1_ST.Isaddress_1)
#define pHET_Isaddress_1  	45

#define HET_NotAddress_1	(e_HETPROGRAM1_UN.Program1_ST.NotAddress_1)
#define pHET_NotAddress_1  	46

#define HET_CheckACK_1	(e_HETPROGRAM1_UN.Program1_ST.CheckACK_1)
#define pHET_CheckACK_1  	47

#define HET_ByteRecTran_1	(e_HETPROGRAM1_UN.Program1_ST.ByteRecTran_1)
#define pHET_ByteRecTran_1  	48

#define HET_GenINTNACK_1	(e_HETPROGRAM1_UN.Program1_ST.GenINTNACK_1)
#define pHET_GenINTNACK_1  	49

#define HET_CheckLast_1	(e_HETPROGRAM1_UN.Program1_ST.CheckLast_1)
#define pHET_CheckLast_1  	50

#define HET_Islast_1	(e_HETPROGRAM1_UN.Program1_ST.Islast_1)
#define pHET_Islast_1  	51

#define HET_CheckRW2_1	(e_HETPROGRAM1_UN.Program1_ST.CheckRW2_1)
#define pHET_CheckRW2_1  	52

#define HET_Isread_1	(e_HETPROGRAM1_UN.Program1_ST.Isread_1)
#define pHET_Isread_1  	53

#define HET_CheckStop_1	(e_HETPROGRAM1_UN.Program1_ST.CheckStop_1)
#define pHET_CheckStop_1  	54

#define HET_StopYes_1	(e_HETPROGRAM1_UN.Program1_ST.StopYes_1)
#define pHET_StopYes_1  	55

#define HET_GoStat0_1	(e_HETPROGRAM1_UN.Program1_ST.GoStat0_1)
#define pHET_GoStat0_1  	56

#define HET_Check_Stat7_1	(e_HETPROGRAM1_UN.Program1_ST.Check_Stat7_1)
#define pHET_Check_Stat7_1  	57

#define HET_Stat7BR_1	(e_HETPROGRAM1_UN.Program1_ST.Stat7BR_1)
#define pHET_Stat7BR_1  	58

#define HET_Check_Stat8_1	(e_HETPROGRAM1_UN.Program1_ST.Check_Stat8_1)
#define pHET_Check_Stat8_1  	59

#define HET_Stat8BR_1	(e_HETPROGRAM1_UN.Program1_ST.Stat8BR_1)
#define pHET_Stat8BR_1  	60

#define HET_Check_Stat9_1	(e_HETPROGRAM1_UN.Program1_ST.Check_Stat9_1)
#define pHET_Check_Stat9_1  	61

#define HET_Stat9BR_1	(e_HETPROGRAM1_UN.Program1_ST.Stat9BR_1)
#define pHET_Stat9BR_1  	62

#define HET_State9_1	(e_HETPROGRAM1_UN.Program1_ST.State9_1)
#define pHET_State9_1  	63

#define HET_TimeOut9CNT_1	(e_HETPROGRAM1_UN.Program1_ST.TimeOut9CNT_1)
#define pHET_TimeOut9CNT_1  	64

#define HET_TimeOut9OCU_1	(e_HETPROGRAM1_UN.Program1_ST.TimeOut9OCU_1)
#define pHET_TimeOut9OCU_1  	65

#define HET_Check_Stat10_1	(e_HETPROGRAM1_UN.Program1_ST.Check_Stat10_1)
#define pHET_Check_Stat10_1  	66

#define HET_Stat10BR_1	(e_HETPROGRAM1_UN.Program1_ST.Stat10BR_1)
#define pHET_Stat10BR_1  	67

#define HET_SCL_SHFT_1	(e_HETPROGRAM1_UN.Program1_ST.SCL_SHFT_1)
#define pHET_SCL_SHFT_1  	68

#define HET_SDA_SHFT_1	(e_HETPROGRAM1_UN.Program1_ST.SDA_SHFT_1)
#define pHET_SDA_SHFT_1  	69

#define HET_CurrSTAT_1	(e_HETPROGRAM1_UN.Program1_ST.CurrSTAT_1)
#define pHET_CurrSTAT_1  	70

#define HET_TimeOUTOCU_1	(e_HETPROGRAM1_UN.Program1_ST.TimeOUTOCU_1)
#define pHET_TimeOUTOCU_1  	71



typedef union 
{ 
 	HET_MEMORY	Memory1_PST[72];
	struct
	{
		SUB_INSTRUCTION Master_Start_1;
		BR_INSTRUCTION Check_Stat0_1;
		AND_INSTRUCTION State0_1;
		MOV32_INSTRUCTION Clear_Time2_1;
		MOV32_INSTRUCTION Clear_Time6_1;
		MOV32_INSTRUCTION Clear_Time9_1;
		AND_INSTRUCTION NewDataFlag_1;
		BR_INSTRUCTION CheckNewData_1;
		MOV32_INSTRUCTION ClearREM_1;
		AND_INSTRUCTION CheckTranInt_1;
		BR_INSTRUCTION GeneTranInt_1;
		MOV32_INSTRUCTION SCLWave_1;
		AND_INSTRUCTION GetData_1;
		AND_INSTRUCTION CheckStart_1;
		BR_INSTRUCTION StartYes_1;
		OR_INSTRUCTION AddStartBit_1;
		MOV32_INSTRUCTION RestoreRead_1;
		XOR_INSTRUCTION InvR_W_1;
		AND_INSTRUCTION CpyR_W_1;
		AND_INSTRUCTION CpyBytes_1;
		OR_INSTRUCTION NoStartBit_1;
		MOV32_INSTRUCTION GoStat4_1;
		SUB_INSTRUCTION Check_Stat1_1;
		BR_INSTRUCTION Stat1BR_1;
		SUB_INSTRUCTION Check_Stat2_1;
		BR_INSTRUCTION Stat2BR_1;
		BR_INSTRUCTION State2_1;
		CNT_INSTRUCTION TimeOut2CNT_1;
		BR_INSTRUCTION TimeOut2OCU_1;
		SUB_INSTRUCTION Check_Stat3_1;
		BR_INSTRUCTION Stat3BR_1;
		SUB_INSTRUCTION Check_Stat4_1;
		BR_INSTRUCTION Stat4BR_1;
		SUB_INSTRUCTION Check_Stat5_1;
		BR_INSTRUCTION Stat5BR_1;
		SUB_INSTRUCTION Check_Stat6_1;
		BR_INSTRUCTION Stat6BR_1;
		BR_INSTRUCTION State6_1;
		CNT_INSTRUCTION TimeOut6CNT_1;
		BR_INSTRUCTION TimeOut6OCU_1;
		SHFT_INSTRUCTION ShiftBitIn_1;
		CNT_INSTRUCTION BitCNT_1;
		BR_INSTRUCTION ByteReady_1;
		MOV32_INSTRUCTION NextData_1;
		OR_INSTRUCTION CleanRecDat_1;
		OR_INSTRUCTION Isaddress_1;
		BR_INSTRUCTION NotAddress_1;
		AND_INSTRUCTION CheckACK_1;
		DJZ_INSTRUCTION ByteRecTran_1;
		BR_INSTRUCTION GenINTNACK_1;
		OR_INSTRUCTION CheckLast_1;
		MOV32_INSTRUCTION Islast_1;
		AND_INSTRUCTION CheckRW2_1;
		MOV32_INSTRUCTION Isread_1;
		AND_INSTRUCTION CheckStop_1;
		BR_INSTRUCTION StopYes_1;
		MOV32_INSTRUCTION GoStat0_1;
		SUB_INSTRUCTION Check_Stat7_1;
		BR_INSTRUCTION Stat7BR_1;
		SUB_INSTRUCTION Check_Stat8_1;
		BR_INSTRUCTION Stat8BR_1;
		SUB_INSTRUCTION Check_Stat9_1;
		BR_INSTRUCTION Stat9BR_1;
		BR_INSTRUCTION State9_1;
		CNT_INSTRUCTION TimeOut9CNT_1;
		BR_INSTRUCTION TimeOut9OCU_1;
		SUB_INSTRUCTION Check_Stat10_1;
		BR_INSTRUCTION Stat10BR_1;
		SHFT_INSTRUCTION SCL_SHFT_1;
		SHFT_INSTRUCTION SDA_SHFT_1;
		CNT_INSTRUCTION CurrSTAT_1;
		MOV32_INSTRUCTION TimeOUTOCU_1;
	} Program1_ST; 

} HETPROGRAM1_UN;

extern volatile HETPROGRAM1_UN e_HETPROGRAM1_UN;

extern const HET_MEMORY HET_INIT1_PST[72];

#endif

