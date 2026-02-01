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

char *next_token(char **str);
uint16_t parseNumber(char **str);
uint32_t parseNumber32(char **str);

int parse_uint8(char **str, uint8_t *num, int base);
int parse_uint16(char **str, uint16_t *num, int base);
int parse_uint32(char **str, uint32_t *num, int base);
int parse_freq(char **str, uint32_t *num);
int parse_bool(char **str, bool *val);
int parse_uint16_range(char **str, uint16_t *num1, uint16 *num2, int base);

void printID(void);


#define COM_STRING_SIZE 90

#endif /* UTILITIES_INC_CONSOLEROUTINES_H_ */
