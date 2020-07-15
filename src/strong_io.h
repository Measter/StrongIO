#ifndef STRONG_IO_H
#define STRONG_IO_H

#include "strong_periphs.h"
#include "strong_pins.h"
#include "common.h"

struct Pullup {
public:
    static constexpr bool is_pullup = true;
};
struct NoPullup {
    static constexpr bool is_pullup = false;
};

enum class PinState {
    Low,
    High,
};

inline PinState operator! (PinState p) {
    if (p == PinState::Low)
        return PinState::High;
    return PinState::Low;
}

namespace IO {

    template<class Pin>
    class DigitalOutBase {
        static_assert(is_digital<Pin>::value, "DigitalOut can only be used with a digital port");
        protected:
            inline DigitalOutBase() {}
            inline void inner_init() {
                Pin::Port::ModeRegister::set_bit(Pin::digital_pin_bit);
            }
        public:
            inline void toggle() {
                Pin::Port::InputRegister::set_bit(Pin::digital_pin_bit);
            }

            inline void write(PinState state) {
                if (state == PinState::Low)
                    set_low();
                else
                    set_high();

                return;
            }

            inline void set_high() {
                Pin::Port::OutputRegister::set_bit(Pin::digital_pin_bit);
            }

            inline void set_low() {
                Pin::Port::OutputRegister::clear_bit(Pin::digital_pin_bit);
            }

            template<class ClockPin>
            inline void shift_out(DigitalOutBase<ClockPin>& clockPin, Peripherals::Serials::SPIBitOrder bitOrder, uint8_t value) {
                static_assert(is_digital<ClockPin>::value, "clockPin must be a DigitalOut");
                // Having the redundant loop is more faster because there's less branching
                // inside the loop.
                if (bitOrder == Peripherals::Serials::SPIBitOrder::LSBFirst) {
                    for (uint8_t i = 0; i < 8; i++) {
                        if (value & 0x1)
                            this->set_high();
                        else
                            this->set_low();
                        
                        clockPin.toggle();
                        clockPin.toggle();
                        value >>= 1;
                    }
                }
                else {
                    for (uint8_t i = 0; i < 8; i++) {
                        if (value & 0x80)
                            this->set_high();
                        else
                            this->set_low();
                        
                        clockPin.toggle();
                        clockPin.toggle();
                        value <<= 1;
                    }
                }
            }
    };

    template<class Pin, class=void>
    class DigitalOut : public DigitalOutBase<Pin> {
        public:
            inline DigitalOut() {
                this->inner_init();
            }
    };

    template<class Pin>
    class DigitalOut<Pin, typename enable_if<has_timer<Pin>::value>::type> : public DigitalOutBase<Pin> {
        public:
            inline DigitalOut() {
                // Ensure the timer is disconnected from the pin.
                Pin::TimerChannel::set_mode(Peripherals::Timers::CompareOutputMode::Mode0);

                this->inner_init();
            }
    };


    template<typename Pin, typename Mode>
    class DigitalInBase {
        static_assert(is_digital<Pin>::value, "DigitalIn can only be used with a digital port");
        protected: 
            inline DigitalInBase() {}
            inline void inner_init() {
                Pin::Port::ModeRegister::clear_bit(Pin::digital_pin_bit);

                if (Mode::is_pullup)
                    Pin::Port::OutputRegister::set_bit(Pin::digital_pin_bit);
                else
                    Pin::Port::OutputRegister::clear_bit(Pin::digital_pin_bit);

                // Datasheet says there should be a NOP after configuring the pin.
                __asm__ volatile ("nop");
            }
        public:
            inline PinState read() {
                if (Pin::Port::InputRegister::get_bit(Pin::digital_pin_bit) != 0)
                    return PinState::High;
                return PinState::Low;
            }

            template<class ClockPin>
            inline uint8_t shift_in(DigitalOutBase<ClockPin> clockPin, Peripherals::Serials::SPIBitOrder bitOrder) {
                // Having the redundant loop ends up being faster due to less branching inside the loop.
                uint8_t value = 0;
                if (bitOrder == Peripherals::Serials::SPIBitOrder::LSBFirst) {
                    uint8_t bit = 0x1;
                    for (uint8_t i = 0; i < 8; i++) {
                        clockPin.toggle();

                        if (this->read() == PinState::High) {
                            value |= bit;
                        }

                        clockPin.toggle();
                        bit <<= 1;
                    }
                } else {
                    uint8_t bit = 0x80;
                    for (uint8_t i = 0; i < 8; i++) {
                        clockPin.toggle();

                        if (this->read() == PinState::High) {
                            value |= bit;
                        }

                        clockPin.toggle();
                        bit >>= 1;
                    }
                }

                return value;
            }
    };

    template<typename Pin, typename Mode = NoPullup, class=void>
    class DigitalIn : public DigitalInBase<Pin, Mode> {
        public:
            inline DigitalIn() {
                this->inner_init();
            }
    };

    template<typename Pin, typename Mode>
    class DigitalIn<Pin, Mode, typename enable_if<has_timer<Pin>::value>::type> : public DigitalInBase<Pin, Mode> {
        public:
            inline DigitalIn() {
                // Ensure the timer is disconnected from the pin.
                Pin::TimerChannel::set_mode(Peripherals::Timers::CompareOutputMode::Mode0);

                this->inner_init();
            }
    };

    template<typename Pin>
    class AnalogInBase {
        static_assert(is_analog_adc<Pin>::value, "AnalogIn can only be used with an analog pin");
        protected:
            inline AnalogInBase() {}
        public:
            inline void start_read() {
                Peripherals::Analog::AnalogDigitalConverter::set_channel(Pin::analog_channel);
                Peripherals::Analog::AnalogDigitalConverter::start_conversion();
            }

            inline uint16_t finish_read() {
                Peripherals::Analog::AnalogDigitalConverter::wait_for_conversion();
                return Peripherals::Analog::AnalogDigitalConverter::read_data();
            }

            inline uint16_t read() {
                start_read();
                return finish_read();
            }
    };

    template<typename Pin, class=void>
    class AnalogIn : public AnalogInBase<Pin> {
        public:
            inline AnalogIn() {
                return;
            }
    };

    template<typename Pin>
    class AnalogIn<Pin, typename enable_if<is_digital<Pin>::value>::type> : public AnalogInBase<Pin> {
        public:
            inline AnalogIn() {
                // Reset pin to input, no pullup, to ensure no interference.
                AnalogIn<Pin, NoPullup> p;
            }
    };

    template<typename Pin>
    class PWM {
        static_assert(has_timer<Pin>::value, "PWM can only be used with a pin that has a timer");
        public:
            inline PWM() {
                // First configure the pin to output.
                Pin::Port::ModeRegister::set_bit(Pin::digital_pin_bit);

                // We won't enable the timer here.
                // We'll instead do what the Arduino IO does, and conditionally dis/connect
                // it based on whether the input is 0, 255, or between.
            }

            inline PWM(
                typename Pin::TimerChannel::Timer::PrescaleMode prescale,
                typename Pin::TimerChannel::Timer::WaveformMode waveform
            ) {
                // First configure the pin to output.
                Pin::Port::ModeRegister::set_bit(Pin::digital_pin_bit);

                using Timer = typename Pin::TimerChannel::Timer;
                Timer::set_prescale(prescale);
                Timer::set_waveform(waveform);

                // We won't enable the timer here.
                // We'll instead do what the Arduino IO does, and conditionally dis/connect
                // it based on whether the input is 0, 255, or between.
            }

            inline void set_duty(uint8_t duty) {
                if (duty == 0) {
                    // Disconnect timer.
                    Pin::TimerChannel::set_mode(Peripherals::Timers::CompareOutputMode::Mode0);
                    Pin::Port::OutputRegister::clear_bit(Pin::digital_pin_bit);
                } else if (duty == 255) {
                    // Disconnect timer.
                    Pin::TimerChannel::set_mode(Peripherals::Timers::CompareOutputMode::Mode0);
                    Pin::Port::OutputRegister::set_bit(Pin::digital_pin_bit);
                } else {
                    Pin::TimerChannel::set_mode(Peripherals::Timers::CompareOutputMode::Mode2);
                    Pin::TimerChannel::set_output_compare(duty);
                }
            }
    };

    template<typename Positive, typename Negative>
    class AnalogComp {
        static_assert(is_analog_ac_positive<Positive>::value, "Analog Comparitor positive pin must be AIN0");
        static_assert(is_analog_ac_negative<Negative>::value, "Analog Comparitor negative pin must be AIN1");
        public:
            inline AnalogComp() {
                // Set both pins to input, no pullup.
                // On the 328P and 2560 these are both on the same port, so do both pins in one operation.
                Positive::Port::ModeRegister::clear_bits(Positive::digital_pin_bit, Negative::digital_pin_bit);
                Positive::Port::OutputRegister::clear_bits(Positive::digital_pin_bit, Negative::digital_pin_bit);
            }

            inline bool is_positive_higher() {
                return Positive::AnalogCompPositive::is_positive_higher();
            }
    };

    template<typename Pin>
    class Tone {
        static_assert(has_tone<Pin>::value, "Tone can only be used with a timer pin on Channel A");
        public:
            inline void init() {
                // Setting pin to output.
                Pin::Port::ModeRegister::set_bit(Pin::digital_pin_bit);

                using WaveformMode = typename Pin::TimerChannel::Timer::WaveformMode;

                Pin::TimerChannel::Timer::reset_timer_control();
                Pin::TimerChannel::Timer::set_waveform(WaveformMode::CTC_OCRA);
            }

            inline Tone() {
                this->init();
            }

            inline void set_tone(uint16_t frequency) {
                using Timer = typename Pin::TimerChannel::Timer;
                using PrescaleMode = typename Pin::TimerChannel::Timer::PrescaleMode;

                frequency *= 2;
                uint16_t target_prescale = F_CPU / 256 / frequency;
                uint8_t mode;
                for (mode = 0; mode < Timer::prescale_count; mode++) {
                    if (pgm_read_word(Timer::prescale_values() + mode) >= target_prescale) {
                        break;
                    }
                }

                uint8_t ocr = F_CPU / pgm_read_word(Timer::prescale_values() + mode) / frequency - 1;
                Pin::TimerChannel::set_output_compare(ocr);
                Pin::TimerChannel::Timer::set_prescale(static_cast<PrescaleMode>(mode+1));

                // Connect Pin to timer to toggle on each compare match.
                Pin::TimerChannel::set_mode(Peripherals::Timers::CompareOutputMode::Mode1);
            }

            inline void stop_tone() {
                // Just disconnect the pin. We won't bother turning the timer off.
                Pin::TimerChannel::set_mode(Peripherals::Timers::CompareOutputMode::Mode0);
            }
    };

    // This type is based off the timing functions from the Arduino library.
    template<typename Timer, typename IntType = uint32_t>
    class Clock {
        private:
            static constexpr uint32_t ClockCyclesPerMicro = F_CPU / 1000000L;

            inline static constexpr uint32_t clock_cycles_to_micros(uint32_t a) {
                return a / ClockCyclesPerMicro;
            }

            static constexpr uint32_t MicrosPerOverflow = clock_cycles_to_micros(64 * 256);
            static constexpr uint32_t MillisPerOverflow = MicrosPerOverflow / 1000;
            static constexpr uint8_t FractionalMillisPerOverflow = (MicrosPerOverflow % 1000) >> 3;
            static constexpr uint8_t FractMax = 1000 >> 3;

            IntType timer_overflow_count;
            IntType timer_millis;
            uint8_t timer_fract;

        public:
            inline Clock(bool configure_timer = false) {
                if (configure_timer) {
                    using PrescaleMode = typename Timer::PrescaleMode;
                    using WaveformMode = typename Timer::WaveformMode;
                    Timer::set_prescale(PrescaleMode::PS64);
                    Timer::set_waveform(WaveformMode::PWMFast_MAX);
                    Timer::enable_overflow_interrupt();
                }
                this->reset();
            }

            inline void reset() {
                this->timer_overflow_count = 0;
                this->timer_millis = 0;
                this->timer_fract = 0;
            }

            inline IntType millis() {
                // Disable interrupts to make sure an interrupt doesn't fire while we're reading
                // the value.
                cli();
                IntType m = this->timer_millis;
                sei();
                return m;
            }

            inline IntType micros() {
                // Disable interrupts to make sure an interrupt doesn't fire in
                // the middle of reading the value.
                cli();
                IntType m = this->timer_overflow_count;
                // We're fine to truncate this, as we assume 256 is the max.
                uint8_t t = Timer::CounterValue::get_value();

                // Note: Possible issue in future? Maybe other AVRs put the TOV bit elsewhere?
                if ((Timer::InterruptFlag::get_bit(TOV0) != 0) && (t < 255)) {
                    m++;
                }
                // Restore interrupts
                sei();

                return ((m << 8) + t) * (64 / ClockCyclesPerMicro);
            }

            inline void delay_millis(IntType ms) {
                IntType start = this->micros();

                while (ms > 0) {
                    while (ms > 0 && (this->micros() - start) >= 1000) {
                        ms--;
                        start += 1000;
                    }
                }
            }

            // This function is copied whole-sale from the Arduino library.
            /* Delay for the given number of microseconds.  Assumes a 1, 8, 12, 16, 20 or 24 MHz clock. */
            void delay_micros(uint16_t us)
            {
                // call = 4 cycles + 2 to 4 cycles to init us(2 for constant delay, 4 for variable)

                // calling avrlib's delay_us() function with low values (e.g. 1 or
                // 2 microseconds) gives delays longer than desired.
                //delay_us(us);
            #if F_CPU >= 24000000L
                // for the 24 MHz clock for the aventurous ones, trying to overclock

                // zero delay fix
                if (!us) return; //  = 3 cycles, (4 when true)

                // the following loop takes a 1/6 of a microsecond (4 cycles)
                // per iteration, so execute it six times for each microsecond of
                // delay requested.
                us *= 6; // x6 us, = 7 cycles

                // account for the time taken in the preceeding commands.
                // we just burned 22 (24) cycles above, remove 5, (5*4=20)
                // us is at least 6 so we can substract 5
                us -= 5; //=2 cycles

            #elif F_CPU >= 20000000L
                // for the 20 MHz clock on rare Arduino boards

                // for a one-microsecond delay, simply return.  the overhead
                // of the function call takes 18 (20) cycles, which is 1us
                __asm__ __volatile__ (
                    "nop" "\n\t"
                    "nop" "\n\t"
                    "nop" "\n\t"
                    "nop"); //just waiting 4 cycles
                if (us <= 1) return; //  = 3 cycles, (4 when true)

                // the following loop takes a 1/5 of a microsecond (4 cycles)
                // per iteration, so execute it five times for each microsecond of
                // delay requested.
                us = (us << 2) + us; // x5 us, = 7 cycles

                // account for the time taken in the preceeding commands.
                // we just burned 26 (28) cycles above, remove 7, (7*4=28)
                // us is at least 10 so we can substract 7
                us -= 7; // 2 cycles

            #elif F_CPU >= 16000000L
                // for the 16 MHz clock on most Arduino boards

                // for a one-microsecond delay, simply return.  the overhead
                // of the function call takes 14 (16) cycles, which is 1us
                if (us <= 1) return; //  = 3 cycles, (4 when true)

                // the following loop takes 1/4 of a microsecond (4 cycles)
                // per iteration, so execute it four times for each microsecond of
                // delay requested.
                us <<= 2; // x4 us, = 4 cycles

                // account for the time taken in the preceeding commands.
                // we just burned 19 (21) cycles above, remove 5, (5*4=20)
                // us is at least 8 so we can substract 5
                us -= 5; // = 2 cycles,

            #elif F_CPU >= 12000000L
                // for the 12 MHz clock if somebody is working with USB

                // for a 1 microsecond delay, simply return.  the overhead
                // of the function call takes 14 (16) cycles, which is 1.5us
                if (us <= 1) return; //  = 3 cycles, (4 when true)

                // the following loop takes 1/3 of a microsecond (4 cycles)
                // per iteration, so execute it three times for each microsecond of
                // delay requested.
                us = (us << 1) + us; // x3 us, = 5 cycles

                // account for the time taken in the preceeding commands.
                // we just burned 20 (22) cycles above, remove 5, (5*4=20)
                // us is at least 6 so we can substract 5
                us -= 5; //2 cycles

            #elif F_CPU >= 8000000L
                // for the 8 MHz internal clock

                // for a 1 and 2 microsecond delay, simply return.  the overhead
                // of the function call takes 14 (16) cycles, which is 2us
                if (us <= 2) return; //  = 3 cycles, (4 when true)

                // the following loop takes 1/2 of a microsecond (4 cycles)
                // per iteration, so execute it twice for each microsecond of
                // delay requested.
                us <<= 1; //x2 us, = 2 cycles

                // account for the time taken in the preceeding commands.
                // we just burned 17 (19) cycles above, remove 4, (4*4=16)
                // us is at least 6 so we can substract 4
                us -= 4; // = 2 cycles

            #else
                // for the 1 MHz internal clock (default settings for common Atmega microcontrollers)

                // the overhead of the function calls is 14 (16) cycles
                if (us <= 16) return; //= 3 cycles, (4 when true)
                if (us <= 25) return; //= 3 cycles, (4 when true), (must be at least 25 if we want to substract 22)

                // compensate for the time taken by the preceeding and next commands (about 22 cycles)
                us -= 22; // = 2 cycles
                // the following loop takes 4 microseconds (4 cycles)
                // per iteration, so execute it us/4 times
                // us is at least 4, divided by 4 gives us 1 (no zero delay bug)
                us >>= 2; // us div 4, = 4 cycles
                

            #endif

                // busy wait
                __asm__ __volatile__ (
                    "1: sbiw %0,1" "\n\t" // 2 cycles
                    "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
                );
                // return = 4 cycles
            }

            inline void tick() {
                IntType m = this->timer_millis;
                uint8_t f = this->timer_fract;

                m += MillisPerOverflow;
                f += FractionalMillisPerOverflow;
                if (f >= FractMax) {
                    f -= FractMax;
                    m += 1;
                }

                this->timer_fract = f;
                this->timer_millis = m;
                this->timer_overflow_count++;
            }
    };
}

#endif //STRONG_IO_H