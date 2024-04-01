.section .init
.globl _start
_start:
   ldr r0, =0x20200000
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
   cmp r3, #1
   beq led_off
   add r3, r3, #1
   str r1, [r0, #0x1C]
   b delay
led_off:
   sub r3, r3, #1
   str r1, [r0, #0x28]
   b delay 
