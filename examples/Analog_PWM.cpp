/*
    Analog/PWM

    Demonstrate the analog and PWM functionality by controlling
    the brightness of an LED from an analog input.
*/

#include <strong_io.h>

int main() {
    // Simply defining the pins here will configure them, so we don't
    // need to do anything more.
    IO::AnalogIn<Pin::A0> sensor_pin;
    IO::PWM<Pin::D11>     led_pin;

    while (true) {

        // Reading an analog input is technically two stages:
        // First you tell the ADC to read, then you wait until
        // it signals that it's finished to get the input.
        //
        // As far as I can tell, there's no reason you couldn't
        // do other things while you wait, so I've split reading
        // into two functions start_read() and finish_read().
        //
        // The read() function used here simply calls both in
        // succession.

        uint16_t sensor_value = sensor_pin.read();
        led_pin.set_duty(sensor_value / 4);
    }
}