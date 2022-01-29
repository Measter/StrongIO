/*
    Timer Configuration

    Quick demonstration of configuring the timers for PWM.
*/

#include <strong_io.h>

int main() {
    using namespace Peripherals::Timers;

    // Using the second constructor on the PWM type to ensure the timer is configured
    // how we expect.
    IO::PWM<Pin::D5> pin1(Timer01PrescaleMode::PS64, Timer02WaveformMode::PWMFast_MAX);
    pin1.set_duty(35);

    // The PWM type only supports 8-bit duty values. Let's go a level down and configure
    // Timer 1 for 10-bit Phase Correct PWM, with an 8 prescale, and a 63% duty,
    // all using the aliases on the pin.
    // The pin needs to be set to output first.
    IO::DigitalOut<Pin::D9> pin2;
    using Pin2Channel = Pin::D9::TimerChannel;
    using Pin2Timer   = Pin::D9::TimerChannel::Timer;

    Pin2Timer::set_waveform(Pin2Timer::WaveformMode::PWM10bitPhaseCorrect);
    Pin2Timer::set_prescale(Pin2Timer::PrescaleMode::PS8);
    // Connects the timer to the pin in non-inverting mode.
    Pin2Channel::set_mode(CompareOutputMode::Mode2);
    Pin2Channel::set_output_compare(645); // 63% duty.
}