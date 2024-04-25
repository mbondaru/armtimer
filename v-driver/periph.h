#ifndef PERIPH_H_
#define PERIPH_H_

#include "msp432p401r.h"

void start_timer2();
void stop_timer2();
void reset_timer2();
void initialize_timer2();
void configure_hfxtclk();
void init_pins();

#endif /* PERIPH_H_ */
