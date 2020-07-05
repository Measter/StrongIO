#ifndef STRONG_PERIPHERALS_H
#define STRONG_PERIPHERALS_H

#if defined (__AVR_ATmega328P__)
#  include <periphs/strong_periphs_328p.h>
#else
#  error "MCU not supported"
#endif

#endif // STRONG_PERIPHERALS_H
