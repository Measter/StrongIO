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
configuring the pins without noticing until I was writing this readme. The incorrect code was accepted and ran without
any indication that it was incorrect.

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

# API

## Pins

The pins are made available in the `Pin` namespace, and have Arduino-style names. The following list gives the pins
available for each function:

* Digital In/Out: `D0`, `D1`, `D2`, `D3`, `D4`, `D5`, `D6`, `D7`, `D8`, `D9`, `D10`, `D11`, `D12`, `D13` (alias `LedBuiltin`),
`A0`, `A1`, `A2`, `A3`, `A4`, `A5`
* Analog Input: `A0`, `A1`, `A2`, `A3`, `A4`, `A5`, `A6`, `A7`
* PWM: `D3`, `D5`, `D6`, `D9`, `D10`, `D11`
* Analog Comparitor: `D6`, `D7`
* Tone: `D6`, `D9`, `D11`


Additionally, for easy interop with Arduino libraries expecting integer pin IDs, all pins have an associated `id` field
accessable (e.g. `Pin::D10::id`).

## Timers

There is a fairly basic API for configuring the waveform generator and prescale value of the timers. The available timers
are:

* `Timer0`
* `Timer1`
* `Timer2`

### `TimerN`

Represents the given timer, and contains pointers to all the registers for configuring the timer.

`TimerN::reset_timer_control()`

Resets the control registers (e.g. `TCCR1A`) back to 0. The other timer registers are untouched.

`TimerN::set_prescale(PrescaleMode)`

Sets the prescale mode of the timer. As the available modes of the prescaler differ depending on the timer, the exact
type taken is not the same for all timers:

* `Timer0`, and `Timer1` take `Timer01PrescaleMode`
* `Timer2` takes `Timer2PrescaleMode`

`TimerN::set_waveform(WaveformMode)`

Sets the waveform mode of the timer. As with the prescale, the available modes differ depending on the timer, as does
the input type::

* `Timer0` and `Timer2` take `Timer02WaveformMode`
* `Timer1` takes `Timer1WaveformMode`

### Example
```
// Configuring Timer1 as the Arduino environment does.
Timers::Timer1 timer;
timer.set_waveform(Timers::Timer1WaveformMode::PWMPhaseCorrect_MAX);
timer.set_prescale(Timers::Timer01PrescaleMode::PS64);
```

---

### `ChannelA<Timer>` and `ChannelB<Timer>`

* `Timer`: One of the timers on the chip, as described above.

`ChannelN::set_mode(CompareOutputMode)`

Sets the mode of the channel.

`ChannelN::set_output_compare(Timer::CompareType value)`

Sets the output compare register for the channel. `CompareType` is `uint8_t` for `Timer0` and `Timer2`, `uint16_t`
for `Timer1`.

---

### `Timer01PrescaleMode`

Represents the available prescale modes for `Timer0` and `Timer1`. Variants are:

* `Stopped`
* `PS1`
* `PS8`
* `PS64`
* `PS256`
* `PS1024`
* `ExternalFalling`
* `ExternalRising`

---

### `Timer2PrescaleMode`

Represents the available prescale modes for `Timer2`. Variants are:

* `Stopped`
* `PS1`
* `PS8`
* `PS32`
* `PS64`
* `PS128`
* `PS256`
* `PS1024`

---

### `Timer02WaveformMode`

Represents the waveform modes awailable for `Timer0` and `Timer2`. The `MAX` and `OCRA` are the `TOP` value.

* `Normal`
* `PWMPhaseCorrect_MAX`
* `CTC_OCRA`
* `PWMFast_MAX`
* `PWMPhaseCorrect_OCRA`
* `PWMFast_OCRA`

---

### `Timer1WaveformMode`

Represents the available waveform modes for `Timer1`. The `MAX`, `OCRA` and `ICR` are the `TOP` values. Note that the 
8-bit modes have the same name as in `Timer02WaveformMode` for consistancy.

* `Normal`
* `PWMPhaseCorrect_MAX`
* `PWM9bitPhaseCorrect`
* `PWM10bitPhaseCorrect`
* `CTC_OCRA`
* `PWMFast_MAX`
* `PWM9bitFast`
* `PWM10bitFast`
* `PWMPhaseFreqCorrect_ICR`
* `PWMPhaseFreqCorrect_OCRA`
* `PWMPhaseCorrect_ICR`
* `PWMPhaseCorrect_OCRA`
* `CTC_ICR`
* `PWMFast_ICR`
* `PWMFast_OCRA`

---

### `CompareOutputMode`

Represents the available compare output mode for the channel. These are generically named due to the exact meaning of
the mode depending on the timer's waveform mode.

* `Mode0`
* `Mode1`
* `Mode2`
* `Mode3`

---

## IO

### `PinState`

An enum class used to represent whether a pin is `High` or `Low`.

---

### `BitOrder`

An enum class for defining the bit order in shifting operations. Variants are `MSBFirst` and `LSBFirst`.

---

### `IO::DigitalIn<Pin, Mode>`

* `Pin`: An IO pin capable of digital input (e.g. `Pin::D4`, `Pin::A1`, etc.). Cannot be `Pin::A6` or `Pin::A7`.
* `Mode`: Whether the internal pullup resistor is connected. Possible values are `Pullup` and `NoPullup`. If not
provided, defaults to `NoPullup`.

`PinState DigitalIn::read()`

Reads the current state of the pin, returning it as a `PinState`.

`uint8_t DigitalIn::shift_in(DigitalOut<ClockPin>, BitOrder)`

Reads in 8 bits of data from the pin by toggling the clock pin to the active state, reading the input pin, then toggling
the clock pin again. Clock pin should be in the inactive state before calling.

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

* `Pin`: An IO pin capable of digital output. (e.g. `Pin::D4`, `Pin::A1`, etc.). Cannot be `Pin::A6` or `Pin::A7`.

`DigitalOut::write(PinState)`

Sets the pin to the given state.

`DigitalOut::set_high()` and `DigitalOut::set_low()`

Specifically sets the pin high or low.

`DigitalOut::toggle()`

Toggles the pin from High to Low, or vice-versa.

`DigitalOut::shift_out(DigitalOut<ClockPin>, BitOrder, uint8_t)`

Writes out 8 bits of data to the pin one at a time, toggling the clock pin twice after each write. Clock pin should be
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

* `Pin`: An IO pin connected to a timer. Available pins are `D3`, `D5`, `D6`, `D9`, `D10`, and `D11`.

Does not reconfigure timer. The timer is not connected on initialization, only when `set_duty` is called. This type
limits you to 8-bit mode PWM on the 16-bit `Timer1`,

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

* `Pin`: An IO pin connected to the ADC. Available pins are `A0` to `A7`.

`AnalogIn::start_read()`

Configures the ADC and starts the conversion. A value cannot be obtained until the conversion has finished, done by
calling `finish_read`.

`uint16_t AnalogIn::finish_read()`

Waits for the conversion to finish, then fetches the value from the ADC. This and the above function are useful if you 
have other things that can be done while you wait for the conversion.

`uint16_t AnalogIn::read()`

Simply calls the above two functions in succession.

### Example
```
IO::AnalogIn<Pin:A5> sensor;
int value = sensor.read();
```

---

### `IO::analog_reference`

Used to set the voltage reference for the ADC. Default value is the internal reference, but can be set to
`AnalogReference::External`.

Warning to those in the Arduino environment: This does **not** in any way interface with the Arduino lib's
`analogReference()` function.

---

### `IO::AnalogComp<PosPin, NegPin>`

* `PosPin`: The positive pin for the analog comparitor. The only pin usable here is `D6`.
* `NegPin`: The negative pin for the analog comparitor. The only pin usable here is `D7`.

This is a function not provided by the Arduino library. It allows comparisons between voltages on the two pins. The reason
it's templated despite there only being one possible pin for each parameter is so it's clear that the pins are in use.

`bool AnalogComp::is_positive_higher()`

Returns `true` if the voltage on the positive pin is higher than the voltage on the negative pin.

### Example
```
IO::AnalogComp<Pin::D6, Pin::D7> comp;
if (comp.is_positive_higher()) {
    // Do something
}
```

---

### `IO::Tone<Pin>`

* `Pin`: A Pin on the A channel of a timer. Usable pins are `D6`, `D9`, and `D11`.

Configures the timer to produce a tone of the given frequency. This function will reconfigure the timer making the B
channel pin (`D3`, `D5`, and `D10`) unusable for PWM.

Notes for those in the Arduino environment:

* Using this with pin `D6` will interfere with the `millis`, `micros`, `delay` and `delayMicroseconds` functions, as they depend on Timer 0 being in a specific configuration.
* The Arduino library's `tone` function uses Timer 2, so use of this type with pin `D11` will interfere with that.
* If you declare a tone pin as a global, you will need to re-initialise it in the `setup` function, as the Arduino
initialisation will reconfigure the timer after the constructor runs.

`Tone::init()`

Will configure the pin ready for tone output. Only needed if used as a global in the Arduino environment.

`Tone::set_tone(int)`

Produces a tone of the given frequency on the pin.

`Tone::stop_tone()`

Halts the tone on the pin. **Does not restore timer configuration**.

### Example
```
IO::Tone<Pin::D6> tonePin;
tonePin.set_tone(800);
```