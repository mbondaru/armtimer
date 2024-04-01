#ifndef ARMTIMER_H_
#define ARMTIMER_H_

#include "rpi.h"
#include "sys/mman.h"
#include "fcntl.h"
#include "stdlib.h"
#include "unistd.h"
#include "stdio.h"
#include "system_rpi.h"

#ifndef BLOCK_SIZE
#define BLOCK_SIZE (4*1024)
#endif
#define CLEAR_IRQ (1)
void armtimer_init(volatile unsigned **uarmtimer_ptr);
void armtimer_print(ARMTimer_Type *armtimer);
void armtimer_enable(uint32_t ldval, ARMTimer_Type *armtimer);
void armtimer_set_predivider(uint32_t pdiv, ARMTimer_Type *armtimer);
void armtimer_disable(ARMTimer_Type *armtimer);
void armtimer_set_16bit_width(ARMTimer_Type *armtimer);
void armtimer_set_23bit_width(ARMTimer_Type *armtimer);

#endif /* ARMTIMER_H_ */
