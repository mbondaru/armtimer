.section ".text.startup"

.global _start
.global _get_stack_pointer
.global _exception_table
.global _enable_interrupts
.global _setup_fiq

.equ CPSR_MODE_USER, 0x10
.equ CPSR_MODE_FIQ, 0x11
.equ CPSR_MODE_IRQ, 0x12
.equ CPSR_MODE_SVR, 0x13
.equ CPSR_MODE_ABORT, 0x17
.equ CPSR_MODE_HYPERVISOR, 0x1A
.equ CPSR_MODE_UNDEFINED, 0x1B
.equ CPSR_MODE_SYSTEM, 0x1F
.equ CPSR_MODE_MASK, 0x1F

.equ CPSR_IRQ_INHIBIT, 0x80
.equ CPSR_FIQ_INHIBIT, 0x40
.equ CPSR_THUMB, 0x20

.equ MAINID_ARMV6, 0x410FB767
.equ MAINID_ARMV7, 0x410FC073
.equ MAINID_ARMV8, 0x410FD034

.equ GIC_DISTB, 0xFF841000
.equ GICC_PMR, 0x4
.equ GICD_CTRLR, 0x0
.equ GICD_IGROUPR, 0x80
.equ GIC_CPUB_offset, 0x1000

#define PRESCALER_2711 0xFF800008
#define MBOX_2711 0xFF8000CC

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
   beq CountEven
   add r12, r12, #0xFF
   str r12, [r8, #0x28]
   add r11, r11, #1
   str r10, [r8, #0x1C]
   mov r12, #0x100
   add r12, r12, #0x67
   cmp r13, r12
   addlo r13, r13, #1
   subeq r13, r13, r12
   subs pc, lr, #4
CountEven:
   str r12, [r8, #0x1C]
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

_reset_h: .word _reset_
_undefined_instruction_vector_h: .word undefined_instruction_vector
_software_interrupt_vector_h: .word software_interrupt_vector
_prefetch_abort_vector_h: .word prefetch_abort_vector
_data_abort_vector_h: .word data_abort_vector
_unused_handler_h: .word _reset_
_interrupt_vector_h: .word interrupt_vector

_reset_:
   mrs r12, cpsr
   and r12, #CPSR_MODE_MASK

   ldr r11, =_cpsr_startup_mode
   str r12, [r11]
   
   mrc p15, 0, r11, c0, c0, 0
   ldr r10, =MAINID_ARMV6
   cmp r11, r10
   beq _setup_interrupt_table

   cmp r12, #CPSR_MODE_HYPERVISOR
   bne _multicore_park

   mrs r12, cpsr
   bic r12, r12, #CPSR_MODE_MASK
   orr r12, r12, #(CPSR_MODE_SVR | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT)
   msr spsr_cxsf, r12

   add lr, pc, #4
   .word 0xE12EF30E
   .word 0xE160006E

_multicore_park:
   mrc p15, 0, r12, c0, c0, 5
   ands r12, #0x3
   bne _inf_loop

_setup_interrupt_table:
   mov r0, #0x8000
   mov r1, #0x0000
   ldmia r0!, {r2, r3, r4, r5, r6, r7, r8}
   stmia r1!, {r2, r3, r4, r5, r6, r7, r8}
   ldr r2, =_fiq_end
Copy:
   ldr r3, [r0]
   str r3, [r1]
   add r0, r0, #4
   add r1, r1, #4
   cmp r0, r2
   bne Copy
   ldmia r0!, {r2, r3, r4, r5, r6, r7, r8}
   stmia r1!, {r2, r3, r4, r5, r6, r7, r8}
   mov r0, #(CPSR_MODE_IRQ | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT)
   msr cpsr_c, r0
   ldr sp, =0x7000

   mov r0, #(CPSR_MODE_SVR | CPSR_IRQ_INHIBIT | CPSR_FIQ_INHIBIT)
   msr cpsr_c, r0
   ldr sp, =0x8000

   bl _cstartup

_inf_loop:
   b _inf_loop

_cpsr_startup_mode: .word 0x0
_osc: .word 19200000
_value: .word 0x63FFF
_mbox: .word MBOX_2711

_enable_interrupts:
   mrs r0, cpsr
   bic r0, r0, #CPSR_IRQ_INHIBIT
   msr cpsr_c, r0
   cpsie i

   mov pc, lr

_setup_fiq:
   mrs r1, cpsr
   mov r2, #0xD1
   msr cpsr_c, r2
   mov r0, r0
   ldr r8, =0x20200000
   ldr r9, =0x2000B400
   mov r10, #0x28
   mov r11, #0
   mov r12, #0
   mov r13, #0
   ldr r14, =__fiq_exit
   msr cpsr_c, r1
   orr r1, r1, #0x80
   bic r1, r1, #0x40
   msr cpsr_c, r1
   
