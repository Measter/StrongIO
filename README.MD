# StrongIO for the Arduino Nano/Uno

Provides a more strongly-typed interface for the GPIO pins on the Nano/Uno's ATmega 328P. Though I intend to use it on 
Arduinos, none of the code depends on the Arduino library being present. It will work without it, though the `AnalogIn`
and `PWM` types assume that the ADC and timers are configured as they are by the Arduino library.

## Goals

I wrote this library because I wanted to make certain mistakes compile errors instead of unexpected runtime behaviour.
For example using a digital pin for analog input, or trying to use PWM on a pin that's not on a timer.

## How it Works

The way this works is by using C++'s templating engine to define your operations at compile-time. The pins are each
unique types, which store the information needed to manipulate them. In addition to that, the differant configurations
of the pins are also different types, with only the valid operations defined. What this means is that, unlike the
Arduino library, it's harder to accidentally read from a pin configured as output for example.

A side effect of defining things in this way is that everything is known at compile time, making it much smaller and
faster than the Arduino library, which does runtime lookups of the pin data.

Configuring a pin is as simple as declaring an IO type with the pin (e.g. `IO::DigitalOut<Pin::D4> ledPin;`).

## Timing and Code Size Test

As a simple comparison, I [wrote three tests](https://gist.github.com/Measter/c0ffa1b10e1e5a8a009ab72722a43610) to compare
the Arduino IO, StrongIO, and direct port access. The test involved setting pins D0, D1, D2, D4, D7, D8, and D12 to input
then reading the pins, then enabling PWM on pins D5, D6, D9, D10, D3, and D11 and setting them to a 50% duty cycle.

It should be noted that in the first test of the Arduino IO, I'd somehow managed to type OUTPUT instead of INPUT when
configuring the pins without noticing until I was writing this readme.

D13 was used as a kind of "bookend" to the two sections which I used to time them, and in all three tests is configured
the same way using direct port access. Analog input is not tested here as it involves waiting for the MCU's ADC which
completely dominates the timing.

The results of the tests are:

|                   | StrongIO  | Arduino IO    | Direct Port IO |
| ---               | ---       | ---           | --- |
| **RAM**           | 9 Bytes   | 9 Bytes       | 9 Bytes |
| **Flash**         | 714 Bytes | 1214 Bytes    | 588 Bytes |
| **Digital Test**  | 6.42μs    | 60.58μs       | 2.67 μs |
| **PWM Test**      | 3.58μs    | 41.42μs       | 1.53μs |

## Provided API

### `PinState`

An enum class used to represent whether a pin is `High` or `Low`.

---

### `BitOrder`

An enum class for defining the bit order in shifting operations. Variants are `MSBFirst` and `LSBFirst`.

---

### `IO::DigitalIn<Pin, Mode>`

* `Pin`: An IO pin defined in the `Pin` namespace (e.g. `Pin::D4`, `Pin::A1`, etc.). Cannot be `Pin::A6` or `Pin::A7`.
* `Mode`: Whether the internal pullup resistor is connected. Possible values are `Pullup` and `NoPullup`. If not
provided, defaults to `NoPullup`.

`PinState DigitalIn::read()`

Reads the current state of the pin, returning it as a `PinState`.

`uint8_t DigitalIn::shift_in(DigitalOut<ClockPin>, BitOrder)`

Reads in 8 bits of data from the pin by toggling the clock pin to the active state then reading the input pin. Clock pin
should be in the inactive state before calling.

### Example:
```
IO::DigitalIn<Pin::D2> clockPin;
IO::DigitalIn<Pin::D3> dataPin;
IO::DigitalIn<Pin::D4, Pullup>, buttonPin;

if (buttonPin.read() == PinState::High) {
    clockPin.set_low();
    int data = dataPin.shift_in(clockPin, BitOrder::MSBFirst);
}
```

---

### `IO::DigitalOut<Pin>`

* `Pin`: An IO pin defined in the `Pin` namespace (e.g. `Pin::D4`, `Pin::A1`, etc.).

`DigitalOut::write(PinState)`

Sets the pin to the given state.

`DigitalOut::set_high()` and `DigitalOut::set_low()`

Specifically sets the pin high or low.

`DigitalOut::toggle()`

Toggles the pin from High to Low, or vice-versa.

`DigitalOut::shift_out(DigitalOut<ClockPin>, BitOrder, uint8_t)`

Writes out 8 bytes of data to the pin one at a time, toggling the clock pin twice after each write. Clock pin should be
in the inactive state before calling.

### Example
```
IO::DigitalOut<Pin::D2> clockPin;
IO::DigitalOut<Pin::D3> dataPin;

clockPin.set_low();
dataPin.shift_out(clockPin, BitOrder::MSBFirst, 0xAB);
```

---

### `IO::PWM<Pin>`

* `Pin`: An IO pin defined in the `Pin` namespace which is connected to a timer. Available pins are `D3`, `D5`, `D6`,
`D9`, `D10`, and `D11`.

The timer is not connected on initialization, only when `set_duty` is called.

`PWM::set_duty(uint8_t)`

Sets the duty cycle of the timer. If the duty is 0 or 255 the timer is disconnected and the pin is set Low or High
respectively.

### Example
```
IO::PWM<Pin::D3> led;
led.set_duty(65);
```

---

### `IO::AnalogIn<Pin>`

* `Pin`: An IO pin defined in the `Pin` namespace wihch is connected to the ADC. Available pins are `A0` to `A7`.

`AnalogIn::pre_read()`

Configures the ADC and starts the conversion. A value cannot be obtained until the conversion has finished, done by
calling `post_read`.

`uint8_t AnalogIn::post_read()`

Waits for the conversion to finish, then fetches the value from the ADC. This and the above function are useful if you 
have other things that can be done while you wait for the conversion.

`uint8_t AnalogIn::read()`

Simply calls the above two functions in succession.

### Example
```
IO::AnalogIn<Pin:A5> sensor;
int value = sensor.read();
```

---

### `IO::AnalogComp<PosPin, NegPin>`

* `PosPin`: The positive pin for the analog comparitor. The only pin usable here is `D6`.
* `NegPin`: The positive pin for the analog comparitor. The only pin usable here is `D7`.

This is a function not provided by the Arduino library. allows comparisons between voltages on the two pins. The reason
it's templated despite there only being one possible pin for each parameter is so it's clear that the pins are in use.

`bool AnalogComp::is_positive_higher()`

Returns whether the voltage on the positive pin is higher than the voltage on the negative pin.

### Example
```
IO::AnalogComp<Pin::D6, Pin::D7> comp;
if (comp.is_positive_higher()) {
    // Do something
}
```