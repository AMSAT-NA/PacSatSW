/*
 * CANTask.h
 *
 *  Created on: Dec 7, 2025
 *      Author: Corey Minyard  AE5KM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 */

#ifndef CANTASK_H
#define CANTASK_H

#define NUM_CAN_BUSSES 2

void CANInit(void);
void CANTask(void *pvParameters);

bool CANSend(int canNum, int priority, int type, uint32_t id, int dest,
	     uint8_t *msg, unsigned int msglen);
void CANSetMyID(int canNum, int id);
int CANGetMyID(int canNum);

void CANEnableLoopback(int canNum, bool enable);

typedef void (*CANReceiveHandler)(int canNum,
				  int priority, int type, int id,
				  int dest, int src,
				  uint8_t *msg, unsigned int msglen);

void CANRegisterReceiveHandler(int canNum, CANReceiveHandler rxhandler);

#endif /* CANTASK_H */
