.section .init
.align 5
.globl _start
_start:
   mrc p15, #0, r1, c0, c0, #5
   and r1, r1, #3
   cmp r1, #0
   bne _halt

   mov r0, #0x00100000
   mov r1, #0
   ldr r2, =0xFFF00000
   mov r8, #0x4000
   mov r7, #0xC00
mmu_section:
   lsr r3, r1, #20
   lsl r3, r3, #2
   orr r3, r3, r8
   and r4, r1, r2
   orr r4, r4, r7
   orr r4, r4, #0x0A
   str r4, [r3]
   add r1, r1, r0
   cmp r1, r2
   bne mmu_section
   
   ldr r1, =0x3F200000
   lsr r3, r1, #20
   lsl r3, r3, #2
   orr r3, r3, r8
   and r4, r1, r2
   orr r4, r4, r7
   orr r4, r4, #0x02
   str r4, [r3]
  
enable_mmu:
   mov r2, #0
   mov r3, #0x1000
   orr r3, r3, #0x5
   mcr p15, 0, r2, c7, c7, 0
   mcr p15, 0, r2, c8, c7, 0
   mcr p15, 0, r2, c7, c10, 4

   mvn r2, #0
   bic r2, #0x0C
   mcr p15, 0, r2, c3, c0, 0

   mcr p15, 0, r8, c2, c0, 0
   mcr p15, 0, r8, c2, c0, 1

   mrc p15, 0, r2, c1, c0, 0
   orr r2, r2, r3
   mcr p15, 0, r2, c1, c0, 0

enable_caches:
   mrs r0, cpsr
   bic r0, r0, #0x1F
   orr r0, r0, #0x13
   msr spsr_cxsf, r0
   add r0, pc, #4
   msr ELR_hyp, r0
   eret

   mrc p15, 0, r1, c12, c0, 0
   mov r0, #0x8000

   ldmia r0!, {r2, r3, r4, r5, r6, r7, r8, r9}
   stmia r1!, {r2, r3, r4, r5, r6, r7, r8, r9}
   ldmia r0!, {r2, r3, r4, r5, r6, r7, r8, r9}
   stmia r1!, {r2, r3, r4, r5, r6, r7, r8, r9}

   mov r12, #0
   mcr p15, 0, r12, c7, c10, 1
   dsb
   mov r12, #0
   mcr p15, 0, r12, c7, c5, 0
   mov r12, #0
   mcr p15, 0, r12, c7, c5, 6
   dsb
   isb

   mrc p15, 0, r1, c1, c1, 0
   bic r1, r1, #1
   mcr p15, 0, r1, c1, c1, 0

   mrc p15, 0, r2, c1, c0, 0
   orr r2, #0x0004
   orr r2, #0x1000
   orr r2, #0x0800
   mcr p15, 0, r2, c1, c0, 0
 
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
   b halt
