;
; SystemServiceEntry.asm
;
;  Created on: Jun 5, 2019
;      Author: bfisher
;
; This is the handler for the SWI interrupt and the jump table that it uses
; to get the to code implementing the matching system service


	.arm
    .ref  swiWDStart
    .ref  swiPortYield
    .ref  swiPortDisableInterrupts
    .ref  swiWDSetTimer
    .ref  swiPortEnterCritical
    .ref  swiPortExitCritical
    .ref  swiRaisePrivilege
    .ref  swiPortEnableInterrupts
    .ref  swiPortTaskUsesFPU
    .ref  swiBusSwitch

;-------------------------------------------------------------------------------
; SWI Handler, interface to Protected Mode Functions, aka Syscall or system service
; jump table

        .def    vPortSWI
        .asmfunc
vPortSWI
        stmfd   sp!, {r11,r12,lr}
        ldrb    r12, [lr, #-1]
        ldr     r14,  table
        ldr     r12, [r14, r12, lsl #2]
        blx     r12
        ldmfd   sp!, {r11,r12,pc}^

table
        .word    jumpTable

jumpTable
        .word   swiPortYield                  ; 0 - vPortYieldProcessor
        .word   swiRaisePrivilege             ; 1 - Raise Priviledge
        .word   swiPortEnterCritical          ; 2 - vPortEnterCritical
        .word   swiPortExitCritical           ; 3 - vPortExitCritical
        .word   swiPortTaskUsesFPU            ; 4 - vPortTaskUsesFPU
        .word   swiPortDisableInterrupts      ; 5 - vPortDisableInterrupts
        .word   swiPortEnableInterrupts       ; 6 - vPortEnableInterrupts
        .word	swiWDSetTimer				  ; 7 - Set the watchdog timer
        .word   swiWDStart                    ; 8 - Start the watchdog
        .word   swiBusSwitch                  ; 9 - Toggle the error line to switch the bus switch
        .endasmfunc

