.section .init
.align 5
.globl _start
_start:
   ldr pc, =_reset_h
   ldr pc, =_undefined_instruction_vector_h
   ldr pc, =_software_interrupt_vector_h
   ldr pc, =_prefetch_abort_vector_h
   ldr pc, =_data_abort_vector_h
   ldr pc, =_unused_handler_h
   ldr pc, =_interrupt_vector_h
_fiq_start:
   mov r12, #0x04000000
   add r12, r12, #0xFF
   str r12, [r8, #0x28]
   str r10, [r8, #0x1C]
   mov r12, #0x100
   add r12, r12, #0x67
   cmp r13, r12
   addlo r13, r13, #1
   subeq r13, r13, r12
   mov r12, #0x04000000
   str r12, [r8, #0x1C]
   lsl r12, r13, #2
   add r12, r12, r11
   ldr r10, [r12]
   str r12, [r9, #0x0C]
   subs pc, lr, #4
.align 9
SineTable:
.word 0x00000027
.word 0x00000028
.word 0x00000028
.word 0x00000029
.word 0x0000002A
.word 0x0000002A
.word 0x0000002B
.word 0x0000002C
.word 0x0000002C
.word 0x0000002D
.word 0x0000002E
.word 0x0000002E
.word 0x0000002F
.word 0x00000030
.word 0x00000030
.word 0x00000031
.word 0x00000032
.word 0x00000032
.word 0x00000033
.word 0x00000034
.word 0x00000034
.word 0x00000035
.word 0x00000035
.word 0x00000036
.word 0x00000037
.word 0x00000037
.word 0x00000038
.word 0x00000038
.word 0x00000039
.word 0x0000003A
.word 0x0000003A
.word 0x0000003B
.word 0x0000003B
.word 0x0000003C
.word 0x0000003D
.word 0x0000003D
.word 0x0000003E
.word 0x0000003E
.word 0x0000003F
.word 0x0000003F
.word 0x00000040
.word 0x00000040
.word 0x00000041
.word 0x00000041
.word 0x00000042
.word 0x00000042
.word 0x00000043
.word 0x00000043
.word 0x00000044
.word 0x00000044
.word 0x00000044
.word 0x00000045
.word 0x00000045
.word 0x00000046
.word 0x00000046
.word 0x00000046
.word 0x00000047
.word 0x00000047
.word 0x00000048
.word 0x00000048
.word 0x00000048
.word 0x00000049
.word 0x00000049
.word 0x00000049
.word 0x00000049
.word 0x0000004A
.word 0x0000004A
.word 0x0000004A
.word 0x0000004A
.word 0x0000004B
.word 0x0000004B
.word 0x0000004B
.word 0x0000004B
.word 0x0000004C
.word 0x0000004C
.word 0x0000004C
.word 0x0000004C
.word 0x0000004C
.word 0x0000004C
.word 0x0000004C
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004D
.word 0x0000004C
.word 0x0000004C
.word 0x0000004C
.word 0x0000004C
.word 0x0000004C
.word 0x0000004C
.word 0x0000004C
.word 0x0000004B
.word 0x0000004B
.word 0x0000004B
.word 0x0000004B
.word 0x0000004A
.word 0x0000004A
.word 0x0000004A
.word 0x0000004A
.word 0x00000049
.word 0x00000049
.word 0x00000049
.word 0x00000049
.word 0x00000048
.word 0x00000048
.word 0x00000048
.word 0x00000047
.word 0x00000047
.word 0x00000046
.word 0x00000046
.word 0x00000046
.word 0x00000045
.word 0x00000045
.word 0x00000044
.word 0x00000044
.word 0x00000044
.word 0x00000043
.word 0x00000043
.word 0x00000042
.word 0x00000042
.word 0x00000041
.word 0x00000041
.word 0x00000040
.word 0x00000040
.word 0x0000003F
.word 0x0000003F
.word 0x0000003E
.word 0x0000003E
.word 0x0000003D
.word 0x0000003D
.word 0x0000003C
.word 0x0000003B
.word 0x0000003B
.word 0x0000003A
.word 0x0000003A
.word 0x00000039
.word 0x00000038
.word 0x00000038
.word 0x00000037
.word 0x00000037
.word 0x00000036
.word 0x00000035
.word 0x00000035
.word 0x00000034
.word 0x00000034
.word 0x00000033
.word 0x00000032
.word 0x00000032
.word 0x00000031
.word 0x00000030
.word 0x00000030
.word 0x0000002F
.word 0x0000002E
.word 0x0000002E
.word 0x0000002D
.word 0x0000002C
.word 0x0000002C
.word 0x0000002B
.word 0x0000002A
.word 0x0000002A
.word 0x00000029
.word 0x00000028
.word 0x00000028
.word 0x00000027
.word 0x00000026
.word 0x00000026
.word 0x00000025
.word 0x00000024
.word 0x00000024
.word 0x00000023
.word 0x00000022
.word 0x00000022
.word 0x00000021
.word 0x00000020
.word 0x00000020
.word 0x0000001F
.word 0x0000001E
.word 0x0000001E
.word 0x0000001D
.word 0x0000001C
.word 0x0000001C
.word 0x0000001B
.word 0x0000001A
.word 0x0000001A
.word 0x00000019
.word 0x00000018
.word 0x00000018
.word 0x00000017
.word 0x00000017
.word 0x00000016
.word 0x00000015
.word 0x00000015
.word 0x00000014
.word 0x00000013
.word 0x00000013
.word 0x00000012
.word 0x00000012
.word 0x00000011
.word 0x00000011
.word 0x00000010
.word 0x000000F
.word 0x000000F
.word 0x000000E
.word 0x000000E
.word 0x000000D
.word 0x000000D
.word 0x000000C
.word 0x000000C
.word 0x000000B
.word 0x000000B
.word 0x000000A
.word 0x000000A
.word 0x0000009
.word 0x0000009
.word 0x0000009
.word 0x0000008
.word 0x0000008
.word 0x0000007
.word 0x0000007
.word 0x0000007
.word 0x0000006
.word 0x0000006
.word 0x0000005
.word 0x0000005
.word 0x0000005
.word 0x0000004
.word 0x0000004
.word 0x0000004
.word 0x0000003
.word 0x0000003
.word 0x0000003
.word 0x0000003
.word 0x0000002
.word 0x0000002
.word 0x0000002
.word 0x0000002
.word 0x0000001
.word 0x0000001
.word 0x0000001
.word 0x0000001
.word 0x0000001
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000000
.word 0x0000001
.word 0x0000001
.word 0x0000001
.word 0x0000001
.word 0x0000001
.word 0x0000002
.word 0x0000002
.word 0x0000002
.word 0x0000002
.word 0x0000003
.word 0x0000003
.word 0x0000003
.word 0x0000003
.word 0x0000004
.word 0x0000004
.word 0x0000004
.word 0x0000005
.word 0x0000005
.word 0x0000005
.word 0x0000006
.word 0x0000006
.word 0x0000007
.word 0x0000007
.word 0x0000007
.word 0x0000008
.word 0x0000008
.word 0x0000009
.word 0x0000009
.word 0x0000009
.word 0x000000A
.word 0x000000A
.word 0x000000B
.word 0x000000B
.word 0x000000C
.word 0x000000C
.word 0x000000D
.word 0x000000D
.word 0x000000E
.word 0x000000E
.word 0x000000F
.word 0x000000F
.word 0x00000010
.word 0x00000011
.word 0x00000011
.word 0x00000012
.word 0x00000012
.word 0x00000013
.word 0x00000013
.word 0x00000014
.word 0x00000015
.word 0x00000015
.word 0x00000016
.word 0x00000017
.word 0x00000017
.word 0x00000018
.word 0x00000018
.word 0x00000019
.word 0x0000001A
.word 0x0000001A
.word 0x0000001B
.word 0x0000001C
.word 0x0000001C
.word 0x0000001D
.word 0x0000001E
.word 0x0000001E
.word 0x0000001F
.word 0x00000020
.word 0x00000020
.word 0x00000021
.word 0x00000022
.word 0x00000022
.word 0x00000023
.word 0x00000024
.word 0x00000024
.word 0x00000025
.word 0x00000026
SineTableEnd:
_fiq_end:

_undefined_instruction_vector_h: mov pc, lr
_software_interrupt_vector_h: mov pc, lr
_prefetch_abort_vector_h: mov pc, lr
_data_abort_vector_h: mov pc, lr
_unused_handler_h: mov pc, lr
_interrupt_vector_h: mov pc, lr

_reset_h:
   mrc p15, #0, r1, c0, c0, #5
   and r1, r1, #3
   cmp r1, #0
   bne _halt

enable_caches:
@Enter SuperVisor Mode
   mrs r0, cpsr
   bic r0, r0, #0x1F
   orr r0, r0, #0x13
   msr spsr_cxsf, r0
   add r0, pc, #4
   msr ELR_hyp, r0
   eret

@Set VBAR
   mov r0, #0x8000
   mcr p15, 0, r0, c12, c0, 0

@Init caches
   mov r12, #0
   mcr p15, 0, r12, c7, c10, 1
   dsb
   mov r12, #0
   mcr p15, 0, r12, c7, c5, 0
   mov r12, #0
   mcr p15, 0, r12, c7, c5, 6
   dsb
   isb
@INITIALIZE TRANSLATION TABLE
   ldr r1, =0
   ldr r2, =0x4000
   ldr r3, =0x50C0E
1: mov r0, r1, lsl #20
   orr r0, r0, r3
   str r0, [r2], #4
   add r1, r1, #1
   cmp r1, #1008
   bne 1b

   ldr r3, =0x40C06
2: mov r0, r1, lsl #20
   orr r0, r0, r3
   str r0, [r2], #4
   add r1, r1, #1
   cmp r1, #4096
   bne 2b
@Set Secure State
   mrc p15, 0, r1, c1, c1, 0
   bic r1, r1, #1
   mcr p15, 0, r1, c1, c1, 0
@ACTIVATE MMUNU
   mov r0, #0x0
   mcr p15, 0, r0, c2, c0, 2
   mrc p15, 0, r0, c2, c0, 0
   ldr r1, =0x4043
   ldr r2, =0x3FBE
   and r0, r0, r2
   orr r0, r0, r1
   mcr p15, 0, r0, c2, c0, 0
   ldr r0, =0x55555555
   mcr p15, 0, r0, c3, c0, 0
@Turn on instruction, data cache, branch prediction
   mrc p15, 0, r0, c1, c0, 0
   ldr r1, =0xFFFFFFFD
   and r0, r0, r1
   orr r0, r0, #4096
   orr r0, r0, #2048
   orr r0, r0, #4
   mcr p15, 0, r0, c1, c0, 0
@start MMU
   mrc p15, 0, r0, c1, c0, 0
   orr r0, r0, #1
   mcr p15, 0, r0, c1, c0, 0
@Disable Secure State
   mrc p15, 0, r1, c1, c1, 0
   orr r1, r1, #1
   mcr p15, 0, r1, c1, c1, 0
 
_copy_sines:
   mov r2, #0x100
   add r2, r2, #0x68
   mov r0, #0x10000
   ldr r1, =0x8200
   mov r3, #0
_copy_sine:
   lsl r5, r3, #2
   ldrb r4, [r1, r3]
   str r4, [r0, r5]
   add r3, r3, #1
   cmp r3, r2
   bne _copy_sine

_setup_fiq:
   mrs r1, cpsr
   mov r2, #0xD1
   msr cpsr_c, r2
   mov r0, r0
   ldr r8, =0x3F200000
   ldr r9, =0x3F00B400
   ldr r10, =0x26
   ldr r11, =0x8200
   mov r12, #0
   mov r13, #0
   msr cpsr_c, r1
   orr r1, r1, #0x80
   bic r1, r1, #0x40
   msr cpsr_c, r1
_setup_timer:
   ldr r4, =0x3F200000
   ldr r5, =0x3F00B400
   ldr r6, =0x3F00B200
   mov r0, #0
   str r0, [r5, #0x08]
   str r0, [r5, #0x1C]
   str r0, [r6, #0x0C]
   mov r0, #1
   str r0, [r5, #0x0C]
   ldr r0, =0x00249249
   str r0, [r4]
   mov r0, #0x00040000
   str r0, [r4, #0x08]
   ldr r0, =0x040000FF
   str r0, [r4, #0x28]
   mov r0, #13
   str r0, [r5]

_enable_timer_fiq:
   mov r0, #0xC0
   str r0, [r6, #0x0C]
_enable_timer:
   mov r0, #0xA0
   str r0, [r5, #0x08]

_inf_loop:
   b _inf_loop

_halt:
   wfe
   b _halt
