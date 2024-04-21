#ifndef AUX_H_
#define AUX_H_

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

void aux_init(volatile unsigned **uaux_ptr);
void aux_print(AUX_Type *aux);

#endif /* AUX_H_ */
