/*----------------------------------------------------------------------------*/
/* sys_link.cmd                                                               */
/*                                                                            */
/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com  
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "loader_config.h"

/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */

--retain="*(.intvecs)"

/*----------------------------------------------------------------------------*/
/* Memory Map                                                                 */

MEMORY
{

#ifdef USE_BOOTLOADER
    /* For use with the bootloader.  The bootloader will use our vectors at 0x10000 */
    VECTORS  (X)  : origin=0x00010000 length=0x00000020
#else
    /* For use without the bootloader, put our vectors at 0. */
    VECTORS  (X)  : origin=0x00000000 length=0x00000020
#endif
    FLASH0   (RX) : origin=0x00010020 length=0x000AFFE0

    /* Stack is set in sys_core.asm and must be here. */
    STACKS   (RW) : origin=0x08000000 length=0x00001500

    /* Pointer to vector table, used by bootstrap to forward exceptions. */
    RAMVEC   (RW) : origin=0x08001500 length=0x00000004

    /* Area of memory saved across reset.  Used in sys_startup.c */
    SAVEAREA (RW) : origin=0x08001504 length=0x000000fc

    RAM      (RW) : origin=0x08001600 length=0x0001ea00
}

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */

/*
 * Use GROUP to keep the sections in order.  Othersize the linker can change
 * the section order from what is listed.
 */

SECTIONS
{
    .intvecs : {} > VECTORS
    /*
     * .cinit cannot be in the group or you get a link warning about being
     * unable to compress it.  Put it first here and hope for the best.
     */
    .cinit   : {} > FLASH0
    GROUP {
        .text    : {}
        .const   : {}
        .pinit   : {}
        .flash : {
            _flash_last = .;
            . += 4;
            _flash_end = 0xc0000;
        }
    } >FLASH0
    .vecptr  : {} > RAMVEC
    .savearea: {} > SAVEAREA
    GROUP {
        .bss     : {}
        .data    : {}
        .sysmem  : {}
        .heap  : {
            /* Use the rest of RAM for the heap. */
            _heap_start = .;
            . += 4;
            _heap_end = 0x8020000;
        }
    } > RAM
}
