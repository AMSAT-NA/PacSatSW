/*
 * MinMaxCalcs.h
 *
 *  Created on: Jul 27, 2021
 *      Author: bfisher
 */

#ifndef INTERFACES_INC_MINMAXCALCS_H_
#define INTERFACES_INC_MINMAXCALCS_H_


void ClearMinMax ( void );
int getMinMaxResetCount(void);
void MinMaxUpdate(WODHkMRAM_t *buffer);



#endif /* INTERFACES_INC_MINMAXCALCS_H_ */
