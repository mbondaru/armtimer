#ifndef ARMCTL_H_
#define ARMCTL_H_

#include "rpi.h"
#include "sys/mman.h"
#include "unistd.h"
#include "fcntl.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"

#ifndef BLOCK_SIZE
#define BLOCK_SIZE (4*1024)
#endif

#define CORE_TIMER_MAX_DIVIDER ((uint32_t) 0xFFFFFFFF)

void armctl_init(volatile unsigned **uarmctl_ptr);
void armctl_print(ARM_Control_Logic_Module_Type *armctl);
void set_coretimer_divider_maximum(ARM_Control_Logic_Module_Type *armctl);
void set_coretimer_src_crystal(ARM_Control_Logic_Module_Type *armctl);
void set_coretimer_src_apb_clock(ARM_Control_Logic_Module_Type *armctl);
#endif /* ARMCTL_H_ */

