#ifndef CM_H_
#define CM_H_

#include "sys/mman.h"
#include "fcntl.h"
#include "rpi.h"
#include "stdlib.h"
#include "unistd.h"
#include "stdio.h"

#ifndef BLOCK_SIZE
#define BLOCK_SIZE (4*1024)
#endif

void cm_init(volatile unsigned **ucm_ptr);
void cm_print(CM_Type *cm);
void cm_gpclk0_29_5_MHz(CM_Type *cm);
void cm_gpclk0_kill(CM_Type *cm);
void cm_gpclk0_passwd(CM_Type *cm);
#endif /* CM_H */
