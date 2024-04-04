.section .init
.align 5
.globl _start
_start:
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
@Fill VBAR
   mrc p15, 0, r1, c12, c0, 0
   mov r0, #0x8000

   ldmia r0!, {r2, r3, r4, r5, r6, r7, r8, r9}
   stmia r1!, {r2, r3, r4, r5, r6, r7, r8, r9}
   ldmia r0!, {r2, r3, r4, r5, r6, r7, r8, r9}
   stmia r1!, {r2, r3, r4, r5, r6, r7, r8, r9}
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
 
   ldr r0, =0x3F200000
   mov r1, #1
   lsl r1, #18
   str r1, [r0, #0x08]
   mov r1, #0x04000000
   ldr r2, =0x3FFFFFFF
   mov r4, #0
   mov r3, #1
   str r1, [r0, #0x1C]
delay:
   add r4, r4, #1
   cmp r4, r2
   bne delay
blinker:
   mov r4, #0
   cmp r3, #1
   beq led_off
   add r3, r3, #1
   str r1, [r0, #0x1C]
   b delay
led_off:
   sub r3, r3, #1
   str r1, [r0, #0x28]
   b delay

_halt:
   wfe
   b _halt
