.text
.global sp804_handler
.global sp804_handler_end
sp804_handler:
   push {r0-r1}
   ands r0, r11, #1
   beq CountEven
   mov r0, #0x04000000
   add r0, r0, #0xFF
   str r0, [r8, #0x28]
   b CountEvenEnd
SineIndexNotAtEnd:
   str r0, [r8, #0x1C]
   add r12, r12, #1
   b CountEvenEnd
CountEven: 
   mov r0, #0x04000000 
   str r0, [r8, #0x1C] 
   ldr r0, [r10, r12, LSL #2]
   mov r1, #0x67
   add r1, r1, #0x100
   cmp r12, r1
   bmi SineIndexNotAtEnd
   str r0, [r8, #0x1C]
   mov r12, #0
CountEvenEnd:
   add r11, r11, #1
   mov r0, #1
   str r0, [r9, #0x0C]
   pop {r0-r1}
   subs pc, lr, #4
sp804_handler_end: 
