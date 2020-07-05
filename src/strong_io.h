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

enum class BitOrder {
    MSBFirst,
    LSBFirst,
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
            inline void shift_out(DigitalOutBase<ClockPin>& clockPin, BitOrder bitOrder, uint8_t value) {
                static_assert(is_digital<ClockPin>::value, "clockPin must be a DigitalOut");
                // Having the redundant loop is more faster because there's less branching
                // inside the loop.
                if (bitOrder == BitOrder::LSBFirst) {
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
                Pin::TimerChannel::set_mode(Timers::CompareOutputMode::Mode0);

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
            inline uint8_t shift_in(DigitalOutBase<ClockPin> clockPin, BitOrder bitOrder) {
                // Having the redundant loop ends up being faster due to less branching inside the loop.
                uint8_t value = 0;
                if (bitOrder == BitOrder::LSBFirst) {
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
                Pin::TimerChannel::set_mode(Timers::CompareOutputMode::Mode0);

                this->inner_init();
            }
    };

    Analog::VoltageReference analog_reference = Analog::VoltageReference::AVCC;

    template<typename Pin>
    class AnalogInBase {
        static_assert(is_analog_adc<Pin>::value, "AnalogIn can only be used with an analog pin");
        protected:
            inline AnalogInBase() {}
        public:
            inline void start_read() {
                Analog::AnalogDigitalConverter::set_voltage_ref(analog_reference);
                Analog::AnalogDigitalConverter::set_channel(Pin::analog_channel);
                Analog::AnalogDigitalConverter::start_conversion();
            }

            inline uint16_t finish_read() {
                Analog::AnalogDigitalConverter::wait_for_conversion();
                return Analog::AnalogDigitalConverter::read_data();
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
                    Pin::TimerChannel::set_mode(Timers::CompareOutputMode::Mode0);
                    Pin::Port::OutputRegister::clear_bit(Pin::digital_pin_bit);
                } else if (duty == 255) {
                    // Disconnect timer.
                    Pin::TimerChannel::set_mode(Timers::CompareOutputMode::Mode0);
                    Pin::Port::OutputRegister::set_bit(Pin::digital_pin_bit);
                } else {
                    Pin::TimerChannel::set_mode(Timers::CompareOutputMode::Mode2);
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
                Pin::TimerChannel::set_mode(Timers::CompareOutputMode::Mode1);
            }

            inline void stop_tone() {
                // Just disconnect the pin. We won't bother turning the timer off.
                Pin::TimerChannel::set_mode(Timers::CompareOutputMode::Mode0);
            }
    };
}

#endif //STRONG_IO_H