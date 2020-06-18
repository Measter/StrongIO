#include <Arduino.h>
#include "strong_io.h"


void setup() {
    // Basic 8-bit PWM, 30% duty cycle on Channel B, Timer 2.
    IO::PWM<Pin::D3> p;
    p.set_duty(76);

    // Equivalent to this on Channel A, Timer 0:
    IO::DigitalOut<Pin::D6> pwmPin;
    
    using Channel = Pin::D6::TimerChannel;
    // For PWM, Mode2 sets the pin low on a compare match, high on a timer overflow.
    Channel::set_mode(Timers::CompareOutputMode::Mode2);
    Channel::set_output_compare(76);

    // Manually configure Channel A, Timer 1 for 10-bit FastPWM, 8 clock prescale, 30% duty.
    // We can just do this to set the pin as output instead of poking at registers manually.
    IO::DigitalOut<Pin::D9> pwmPin2;
    using Channel2 = Timers::ChannelA<Timers::Timer1>;
    using Timer2 = Timers::Timer1;

    Timer2::set_waveform(Timers::Timer1WaveformMode::PWM10bitFast);
    Timer2::set_prescale(Timers::Timer01PrescaleMode::PS8);
    // // For PWM, Mode2 sets the pin low on a compare match, high on a timer overflow.
    Channel2::set_mode(Timers::CompareOutputMode::Mode2);
    Channel2::set_output_compare(307);
}

void loop() {
}