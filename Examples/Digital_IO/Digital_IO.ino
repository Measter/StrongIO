/* 
    Digital IO

    Example adapted from standard Arduino Debounce example.
*/

#include "strong_io.h"

// Simply defining the pins here will configure them, so we don't
// need to do anything more.
IO::DigitalIn<Pin::D2> buttonPin;
IO::DigitalOut<Pin::D13> ledPin;

PinState buttonState;
PinState lastButtonState = PinState::Low;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
    // There's also a write(PinState) function, but in this case we know
    // at compile time what state we want.
    ledPin.set_high();
}

void loop() {
    PinState reading = buttonPin.read();
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != buttonState) {
            buttonState = reading;

            if (buttonState == PinState::High) {
                // Here I'm exposing a function of the 328P that isn't 
                // obviously exposed in the Arduino IO: you can toggle
                // the pin by writing High to it.
                ledPin.toggle();
            }
        }
    }

    lastButtonState = reading;
}