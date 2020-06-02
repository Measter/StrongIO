/* 
    Analog/PWM

    Demonstrate the analog and PWM functionality by controlling
    the brightness of an LED from an analog input.
*/

#include "strong_io.h"

// Simply defining the pins here will configure them, so we don't
// need to do anything more.
IO::AnalogIn<Pin::A0> sensorPin;
IO::PWM<Pin::D11> ledPin;

void setup() {
}

void loop() {
    // Reading an analog input is technically two stages:
    // First you tell the ADC to read, then you wait until
    // it signals that it's finished to get the input.
    //
    // As far as I can tell, there's no reason you couldn't
    // do other things while you wait, so I've split reading
    // into two functions pre_read() and post_read().
    // 
    // The read() function used here simply calls both in
    // succession.

    uint16_t sensorValue = sensorPin.read();
    ledPin.set_duty(sensorValue/4);
}