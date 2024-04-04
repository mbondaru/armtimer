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
   ands r12, r11, #1
   mov r12, #0x04000000
   str r12, [r9, #0x0C]
   dmb
   beq CountEven
   orr r12, r12, #0xFF
   str r12, [r8, #0x28]
   dmb
   add r11, r11, #1
   str r10, [r8, #0x1C]
   dmb
   mov r12, #0x100
   orr r12, r12, #0x67
   cmp r13, r12
   addlo r13, r13, #1
   subeq r13, r13, r12
   subs pc, lr, #4
CountEven:
   str r12, [r8, #0x1C]
   dmb
   add r11, r11, #1
   adr r12, SineTable
   add r12, r12, r13
   ldrb r10, [r12]
   subs pc, lr, #4
SineTable:
.word 0x28282726
.word 0x2B2A2A29
.word 0x2E2D2C2C
.word 0x30302F2E
.word 0x33323231
.word 0x35353434
.word 0x38373736
.word 0x3A3A3938
.word 0x3D3C3B3B
.word 0x3F3E3E3D
.word 0x4140403F
.word 0x43424241
.word 0x44444443
.word 0x46464545
.word 0x48474746
.word 0x49494848
.word 0x4A4A4949
.word 0x4B4B4A4A
.word 0x4C4C4B4B
.word 0x4C4C4C4C
.word 0x4D4D4D4C
.word 0x4D4D4D4D
.word 0x4D4D4D4D
.word 0x4D4D4D4D
.word 0x4D4D4D4D
.word 0x4C4C4C4C
.word 0x4B4C4C4C
.word 0x4A4B4B4B
.word 0x494A4A4A
.word 0x48494949
.word 0x47474848
.word 0x45464646
.word 0x44444445
.word 0x42424343
.word 0x40404141
.word 0x3E3E3F3F
.word 0x3B3C3D3D
.word 0x393A3A3B
.word 0x37373838
.word 0x34353536
.word 0x32323334
.word 0x2F303031
.word 0x2C2D2E2E
.word 0x2A2A2B2C
.word 0x27282829
.word 0x24252626
.word 0x22222324
.word 0x1F202021
.word 0x1C1D1E1E
.word 0x1A1A1B1C
.word 0x17181819
.word 0x15151617
.word 0x12131314
.word 0x10111112
.word 0xE0E0F0F
.word 0xC0C0D0D
.word 0xA0A0B0B
.word 0x8090909
.word 0x7070708
.word 0x5050606
.word 0x4040405
.word 0x3030303
.word 0x2020202
.word 0x1010101
.word 0x1
.word 0x0
.word 0x0
.word 0x0
.word 0x0
.word 0x0
.word 0x0
.word 0x1010101
.word 0x2020201
.word 0x3030302
.word 0x4040403
.word 0x6050505
.word 0x7070706
.word 0x9090808
.word 0xB0A0A09
.word 0xD0C0C0B
.word 0xF0E0E0D
.word 0x1111100F
.word 0x13131212
.word 0x16151514
.word 0x18181717
.word 0x1B1A1A19
.word 0x1E1D1C1C
.word 0x20201F1E
.word 0x23222221
.word 0x26252424
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
 
_setup_fiq:
   mrs r1, cpsr
   mov r2, #0xD1
   msr cpsr_c, r2
   mov r0, r0
   ldr r8, =0x3F200000
   ldr r9, =0x3F00B400
   mov r10, #0x28
   mov r11, #0
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
   mov r0, #8
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
