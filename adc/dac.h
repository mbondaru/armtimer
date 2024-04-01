#ifndef DAC_H_
#define DAC_H_

#include "math.h"
#include "gpio.h"
#include "cm.h"
#include "armctl.h"
#include "rpi.h"

#define DAC_14BIT_3_3V ((uint16_t) 0x3FFF)
#define DAC_14BIT_1V ((uint16_t) 0x1364)
#define DAC_14BIT_0_5V ((uint16_t) 0x09B2)
#define DAC_MASK ((uint32_t) 0x000000FF)
#define SINE_TABLE_SIZE (360)
#define PI (3.14159265358979323846)
/* Sine table */

#endif /* DAC_H_ */
