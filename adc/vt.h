#ifndef VT_H_
#define VT_H_

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

void vt_init(volatile unsigned **uvtptr);
void vt_print(Vector_Table_Type *fiq);
