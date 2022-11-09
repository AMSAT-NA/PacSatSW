/*
 * serial.h
 *
 *  Created on: Feb 25, 2013
 *      Author: fox
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <pacsat.h>
#include "FreeRTOS.h"


typedef enum
{
  COM1 = 0,
  COM2 = 1,
  COM3 = 2,
} COM_NUM;
#define numCOM 3  /* How many COM ports does this software support? */

/*
 * Call SerialInitPort in to initialize a port before using it.  It only needs to be called
 * once after booting
 */
bool SerialInitPort(COM_NUM comPort, unsigned int baud, unsigned int qLengthRx,unsigned int qLengthTx);

/*
 * The "Put" functions obviously output a character to whichever port is specified
 * by com.  PutChar outputs a single character.  PutString outputs a character string.
 * You can either specify the length if you know it, or specify a length of 0, in which
 * case PutString assumes the string is zero-terminated.
 */
void SerialPutString(COM_NUM com, const char * const string, int length );
bool SerialPutChar( COM_NUM com, char cOutChar, portTickType xBlockTime );
/*
 * The "Get" functions receive data from the serial port specified by com.  GetChar
 * receives a single character.  For the USART ports, characters will be accepted at
 * any time and put into a queue.  GetChar just takes out of the queue (and waits if
 * there is nothing there).
 *
 * If you specify GetBytes, any character in the queue will be put in the buffer first
 * followed by new characters read in up to a total of length.  Note:  If timeout is
 * non-zero, this function will not return until either timeout is exceeded or the
 * specified number of characters are received.  If timeout is specified as 0, that
 * is a special case.  SerialGetBytes will return immediately.  It will return 'True'
 * if there should happen to be length characters already in the queue.  However, if
 * it returns 'false', the data transfer into 'buffer' will keep going in the background.
 * Call SerialWaitBytes to wait until the transfer is complete.
 */
bool SerialGetChar( COM_NUM com, char *pcRxedChar, portTickType xBlockTime );
bool SerialGetBytes(COM_NUM com, char *buffer, int length, portTickType timeout);
bool SerialWaitBytes(COM_NUM com, portTickType timeout);


#endif /* SERIAL_H_ */
