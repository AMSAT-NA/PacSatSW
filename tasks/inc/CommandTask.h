/*
 * CommandTask.h
 *
 *  Created on: Mar 12, 2019
 *      Author: burns
 */

#ifndef TASKS_INC_COMMANDTASK_H_
#define TASKS_INC_COMMANDTASK_H_


void CommandTask(void *pvParameters);


#include <pacsat.h> /* Must precede stdio */
#include "UplinkCommands.h"
#include "canDriver.h"
bool CorrelateSync(uint32_t data);
void NewUplinkByte(uint16_t data);
void RestartUplinkDecode(void);
void NewUplinkBit(uint8_t bit); // Returns true if done with command
void SimulateHWCommand(UplinkCommands cmd);
void SimulateSwCommand(uint8_t namesp, uint16_t cmd, const uint16_t *args,int numargs);

void EnableCommandTimeCheck(bool enable);
void EnableCommandPrint(bool enable);

void NoCommandTimeoutCallback(void);
void NoTimedSWCommandTimeoutCallback(void);
void incomingRawSoftwareCommand(uint8_t *packet);
void incomingRFSoftwareCommand(CANPacket_t *packet);

uint8_t GetHWCmdCount(void);
uint8_t GetSWCmdCount(void);
bool GetVUCDisabled(void);

#define SW_UPLINK_BUFFERS 1



#endif /* TASKS_INC_COMMANDTASK_H_ */
