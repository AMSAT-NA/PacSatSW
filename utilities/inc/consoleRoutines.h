/*
 * consoleRoutines.h
 *
 *  Created on: Sep 21, 2020
 *      Author: bfisher
 */

#ifndef UTILITIES_INC_CONSOLEROUTINES_H_
#define UTILITIES_INC_CONSOLEROUTINES_H_

#include "canID.h"
#include "serialDriver.h"

typedef struct _commandPairs {
    char * typedCommand;
    char * help;
    //CommandIndex
    int indexVal;
} commandPairs;

void printHelp(char *search,commandPairs *commands, int numberOfCommands);
void print8BitTemp(uint8_t temp8);
void DisplayTelemetry(uint32_t typeRequested);
void PreflightInitNow(CanIDNode cameFrom);
void receiveLine(COM_NUM ioCom, char *commandString, char prompt, bool echo);
uint16_t parseNextNumber(void);
uint16_t parseNumber(char *afterCommand);
uint32_t parseNumber32(char *afterCommand);
uint32_t parseNextNumber32(void);
bool AntennaAsk(void);
void receiveLine(COM_NUM ioCom, char *commandString, char prompt, bool echo);

void printID(void);


#define COM_STRING_SIZE 90



#endif /* UTILITIES_INC_CONSOLEROUTINES_H_ */
