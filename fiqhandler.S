.text
.align 5 ; Align code to cache line
.global sp804_handler
.global sp804_handler_end

;setup r13 to point to SineTable
sp804_handler:
   eors r11, r11, #1
   mov r12, #0x04000000
   str r12, [r9, #0x0C]
   beq CountEven
   orr r12, r12, #0xFF
   str r12, [r8, #0x28]
   str r10, [r8, #0x1C] 
   subs pc, lr, #4
CountEven:
   ldrb r10, [r13], #1
   str r12, [r8, #0x1C]
   adr r12, SineTableEnd
   cmp r13, r12
   adreq r13, SineTable
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
SineTableEnd
sp804_handler_end: 
