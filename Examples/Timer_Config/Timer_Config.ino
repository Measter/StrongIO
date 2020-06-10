#include <Arduino.h>
#include "strong_io.h"


void setup() {
    // Basic 8-bit PWM, 30% duty cycle on Channel B, Timer 2.
    IO::PWM<Pin::D3> p;
    p.set_duty(76);

    // Equivalent to this on Channel B, Timer 0:
    IO::DigitalOut<Pin::D6> pwmPin;
    
    typename Pin::D6::TimerChannel channel;
    channel.set_mode(Timers::CompareOutputMode::Mode2);
    channel.set_output_compare(76);

    // Manually configure Channel A, Timer 1 for 10-bit FastPWM, 8 clock prescale, 30% duty.
    IO::DigitalOut<Pin::D9> pwmPin2;
    typename Pin::D9::TimerChannel channel2;
    typename Pin::D9::TimerChannel::Timer timer2;
    using PrescaleMode = Pin::D9::TimerChannel::Timer::PrescaleMode;
    using WaveformMode = Pin::D9::TimerChannel::Timer::WaveformMode;

    timer2.set_waveform(WaveformMode::PWM10bitFast);
    timer2.set_prescale(PrescaleMode::PS8);
    channel2.set_mode(Timers::CompareOutputMode::Mode2);
    channel2.set_output_compare(307);
}

void loop() {
}