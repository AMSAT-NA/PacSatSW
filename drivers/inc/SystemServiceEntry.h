/*
 * SystemServiceEntry.h
 *
 *  Created on: Jun 5, 2019
 *      Author: bfisher
 */

#ifndef DRIVERS_INC_SYSTEMSERVICEENTRY_H_
#define DRIVERS_INC_SYSTEMSERVICEENTRY_H_

/*///////////////////////////
 *
 *  System Service Calls
 *
 * These System Service definitions are for all system services, including
 * those that are defined by FreeRTOS, and which were originally defined in
 * os_portmacro.h.  In all cases, see the jump table in SystemServiceEntry.asm
 */

//////////FreeRTOS definitions (from os_portmacro.h)

/* Floating Point Support */
#pragma SWI_ALIAS(vPortTaskUsesFPU, 4)
extern void vPortTaskUsesFPU(void);

#pragma SWI_ALIAS(vPortYield, 0)
extern void vPortYield( void );


/* Critical section handling. */

#pragma SWI_ALIAS(vPortEnterCritical, 2)
extern void vPortEnterCritical( void );

#pragma SWI_ALIAS(vPortExitCritical, 3)
extern void vPortExitCritical( void );

#pragma SWI_ALIAS(vPortDisableInterrupts, 5)
extern void vPortDisableInterrupts( void );


#pragma SWI_ALIAS(vPortEnableInterrupts, 6)
extern void vPortEnableInterrupts( void );

//////////////////System services added for Golf flight software

#pragma SWI_ALIAS(SWIRaisePrivilege,1)
#pragma SWI_ALIAS(SWISetWDTimer,7)
#pragma SWI_ALIAS(SWIStartWatchdog,8)
#pragma SWI_ALIAS(SWIDoBusSwitch,9)
extern void SWIRaisePrivilege(void);
extern void SWISetWDTimer(void);
extern void SWIStartWatchdog(void);
extern void SWIDoBusSwitch(void);


#endif /* DRIVERS_INC_SYSTEMSERVICEENTRY_H_ */
