#ifndef SYSTIMER_H_
#define SYSTIMER_H_

#include "rpi.h"
#include "sys/mman.h"
#include "fcntl.h"
#include "stdlib.h"
#include "unistd.h"
#include "stdio.h"
#include "system_rpi.h"
#include "armctl.h"

void systimer_init();
void systimer_print();

#endif /* SYSTIMER_H_ */
