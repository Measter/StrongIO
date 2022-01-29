/*
    Clock

    Demonstrates using the clock function with a simple
    button debounce.
*/

#include <strong_io.h>

// Unlike the Arduino environment, You're not forced to use
// any particular timer. The drawback to this is that you must
// handle the tick yourself, as is done here.
IO::Clock<Peripherals::Timers::Timer0> clock(true);

ISR(TIMER0_OVF_vect) {
    clock.tick();
}

int main() {
    // Simply defining the pins here will configure them, so we don't
    // need to do anything more.
    IO::DigitalIn<Pin::D2>   button_pin;
    IO::DigitalOut<Pin::D13> led_pin;

    PinState button_state   = PinState::Low;
    bool     button_changed = false;

    uint32_t last_debounce_time = 0;
    uint32_t debounce_delay     = 10;

    led_pin.set_high();

    while (true) {
        PinState cur_button_state = button_pin.read();
        uint32_t cur_time         = clock.millis();

        if (cur_button_state != button_state && (cur_time - last_debounce_time) >= debounce_delay) {
            button_state   = cur_button_state;
            button_changed = true;
        }

        if (button_changed && button_state == PinState::High) {
            led_pin.toggle();
        }

        button_state = cur_button_state;
    }

    return 0;
}