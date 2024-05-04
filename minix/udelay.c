#include "udelay.h"

void udelay(uint32_t us)
{
  uint32_t current_time = SYSTMR->CLO;
  uint32_t target_time = current_time + us;
  while(SYSTMR->CLO < target_time);
}
