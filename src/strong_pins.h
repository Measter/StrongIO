#ifndef STRONG_PINS_H
#define STRONG_PINS_H

#if defined (__AVR_ATmega328P__)
#  include <pins/strong_pins_328p.h>
#else
#  error "MCU not supported"
#endif

#endif // STRONG_PINS_H
