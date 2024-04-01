#ifndef GPIO_H_
#define GPIO_H_

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
#define GPIO_PIN0 (0)
#define GPIO_PIN1 (1)
#define GPIO_PIN2 (2)
#define GPIO_PIN3 (3)
#define GPIO_PIN4 (4)
#define GPIO_PIN5 (5)
#define GPIO_PIN6 (6)
#define GPIO_PIN7 (7)
#define GPIO_PIN8 (8)
#define GPIO_PIN9 (9)
#define GPIO_PIN10 (10)
#define GPIO_PIN11 (11)
#define GPIO_PIN12 (12)
#define GPIO_PIN13 (13)
#define GPIO_PIN14 (14)
#define GPIO_PIN15 (15)
#define GPIO_PIN16 (16)
#define GPIO_PIN17 (17)
#define GPIO_PIN18 (18)
#define GPIO_PIN19 (19)
#define GPIO_PIN20 (20)
#define GPIO_PIN21 (21)
#define GPIO_PIN22 (22)
#define GPIO_PIN23 (23)
#define GPIO_PIN24 (24)
#define GPIO_PIN25 (25)
#define GPIO_PIN26 (26)
#define GPIO_PIN27 (27)
#define GPIO_PIN28 (28)
#define GPIO_PIN29 (29)
#define GPIO_PIN30 (30)
#define GPIO_PIN31 (31)
#define GPIO_PIN32 (32)
#define GPIO_PIN33 (33)
#define GPIO_PIN34 (34)
#define GPIO_PIN35 (35)
#define GPIO_PIN36 (36)
#define GPIO_PIN37 (37)
#define GPIO_PIN38 (38)
#define GPIO_PIN39 (39)
#define GPIO_PIN40 (40)
#define GPIO_PIN41 (41)
#define GPIO_PIN42 (42)
#define GPIO_PIN43 (43)
#define GPIO_PIN44 (44)
#define GPIO_PIN45 (45)
#define GPIO_PIN46 (46)
#define GPIO_PIN47 (47)
#define GPIO_PIN48 (48)
#define GPIO_PIN49 (49)
#define GPIO_PIN50 (50)
#define GPIO_PIN51 (51)
#define GPIO_PIN52 (52)
#define GPIO_PIN53 (53)

#define GPIO_FUNC_INPUT (0)
#define GPIO_FUNC_OUTPUT (1)
#define GPIO_FUNC_ALT5 (2)
#define GPIO_FUNC_ALT4 (3)
#define GPIO_FUNC_ALT0 (4)
#define GPIO_FUNC_ALT1 (5)
#define GPIO_FUNC_ALT2 (6)
#define GPIO_FUNC_ALT3 (7)
#define GPIO_PINS_PER_FSELN_REGISTER (10)
#define GPIO_FUNC_SEL_FIELD_BITS_COUNT (3)

void pads_init(volatile unsigned **upads_ptr);
void pads_print(GPIO_Pads_Control_Type *pads);
void set_pads_0_27_maximum_drive_strength(GPIO_Pads_Control_Type *pads);
void set_pads_0_27_moderate_drive_strength(GPIO_Pads_Control_Type *pads);
void set_pads_28_45_moderate_drive_strength(GPIO_Pads_Control_Type *pads);
void gpio_init(volatile unsigned **ugpio_ptr);
void gpio_print(GPIO_Type *p1);
void gpio_set_func(uint32_t pin_number, uint32_t alt_func, GPIO_Type *p1);


#endif /* GPIO_H */
