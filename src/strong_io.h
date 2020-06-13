#ifndef STRONG_IO_H
#define STRONG_IO_H

#include "strong_pins.h"
#include "type_traits.h"

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

enum class AnalogReference {
    Default,
    External,
};

namespace IO {

    template<class Pin>
    class DigitalOutBase {
        static_assert(is_digital<Pin>::value, "DigitalOut can only be used with a digital port");
        protected:
            inline DigitalOutBase() {}
            inline void inner_init() {
                typename Pin::Port port;
                *port.mode_register |= Pin::digital_pin_bit;
                // Datasheet says there should be a NOP after configuring the pin for syncronization.
                __asm__ volatile ("nop");
            }
        public:
            inline void toggle() {
                typename Pin::Port port;
                *port.input_register |= Pin::digital_pin_bit;
            }

            inline void write(PinState state) {
                if (state == PinState::Low)
                    set_low();
                else
                    set_high();

                return;
            }

            inline void set_high() {
                typename Pin::Port port;
                *port.output_register |= Pin::digital_pin_bit;
            }

            inline void set_low() {
                typename Pin::Port port;
                *port.output_register &= ~Pin::digital_pin_bit;
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
                // Make sure PWM is disabled.
                typename Pin::TimerChannel channel;
                channel.set_mode(Timers::CompareOutputMode::Mode0);

                this->inner_init();
            }
    };


    template<typename Pin, typename Mode>
    class DigitalInBase {
        static_assert(is_digital<Pin>::value, "DigitalIn can only be used with a digital port");
        protected: 
            inline DigitalInBase() {}
            inline void inner_init() {
                typename Pin::Port port;
                *port.mode_register &= ~Pin::digital_pin_bit;

                if (Mode::is_pullup)
                    *port.output_register |= Pin::digital_pin_bit;
                else
                    *port.output_register &= ~Pin::digital_pin_bit;

                // Datasheet says there should be a NOP after configuring the pin.
                __asm__ volatile ("nop");
            }
        public:
            inline PinState read() {
                typename Pin::Port port;
                if ((*port.input_register & Pin::digital_pin_bit) != 0)
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
                // Make sure PWM is disabled.
                typename Pin::TimerChannel channel;
                channel.set_mode(Timers::CompareOutputMode::Mode0);

                this->inner_init();
            }
    };


    AnalogReference analog_reference = AnalogReference::Default;

    template<typename Pin>
    class AnalogInBase {
        static_assert(is_analog_adc<Pin>::value, "AnalogIn can only be used with an analog pin");
        protected:
            inline AnalogInBase() {}
        public:
            inline void start_read() {
                uint8_t ref = analog_reference == AnalogReference::Default ? 1 << 6 : 0;

                // Configure the ADMUX register for the reference mode and channel we desire.
                ADMUX = ref | Pin::analog_channel;

                // Set the ADSC bit to start our conversation.
                ADCSRA |= (1 << ADSC);
            }

            inline uint16_t finish_read() {
                // Wait until it's finished.
                while(ADCSRA & (1 << ADSC)) {}

                // Now we can read our data out of the registers.
                uint8_t low = ADCL;
                uint8_t high = ADCH;

                return (high << 8) | low;
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
                // Reset pin to tri-state.
                typename Pin::Port port;
                *port.mode_register &= ~Pin::digital_pin_bit;
                *port.output_register &= ~Pin::digital_pin_bit;

                // Datasheet says there should be a NOP after configuring the pin.
                __asm__ volatile ("nop");
                return;
            }
    };

    template<typename Pin>
    class PWM {
        static_assert(has_timer<Pin>::value, "PWM can only be used with a pin that has a timer");
        public:
            inline PWM() {
                // First configure the pin to output.
                typename Pin::Port port;
                *port.mode_register |= Pin::digital_pin_bit;
                // Datasheet says there should be a NOP after configuring the pin for syncronization.
                __asm__ volatile ("nop");

                // We won't enable the timer here.
                // We'll instead do what the Arduino IO does, and conditionally enable/disable
                // it based on whether the input is 0, 255, or between.
            }

            inline void set_duty(uint8_t duty) {
                typename Pin::Port port;
                typename Pin::TimerChannel channel;
                typename Pin::TimerChannel::Timer timer;
                if (duty == 0) {
                    // Disable timer.
                    channel.set_mode(Timers::CompareOutputMode::Mode0);
                    *port.output_register &= ~Pin::digital_pin_bit;
                } else if (duty == 255) {
                    // Disable timer.
                    channel.set_mode(Timers::CompareOutputMode::Mode0);
                    *port.output_register |= Pin::digital_pin_bit;
                } else {
                    channel.set_mode(Timers::CompareOutputMode::Mode2);
                    channel.set_output_compare(duty);
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
                typename Positive::Port port;
                *port.mode_register &= ~(Positive::digital_pin_bit | Negative::digital_pin_bit);
                *port.output_register &= ~(Positive::digital_pin_bit | Negative::digital_pin_bit);
            }

            inline bool is_positive_higher() {
                typename Positive::AnalogCompPositive comp;
                return comp.is_positive_higher();
            }
    };

    template<typename Pin>
    class Tone {
        static_assert(has_tone<Pin>::value, "Tone can only be used with a timer pin on Channel A");
        public:
            inline void init() {
                // Setting pin to output.
                typename Pin::Port port;
                *port.mode_register |= Pin::digital_pin_bit;
                // Shouldn't need a NOP here. Configuring the time will take long enough.

                typename Pin::TimerChannel::Timer timer;
                using WaveformMode = typename Pin::TimerChannel::Timer::WaveformMode;

                timer.reset_timer_control();
                timer.set_waveform(WaveformMode::CTC_OCRA);
            }

            inline Tone() {
                this->init();
            }

            inline void set_tone(uint16_t frequency) {
                typename Pin::TimerChannel channel;
                typename Pin::TimerChannel::Timer timer;
                using PrescaleMode = typename Pin::TimerChannel::Timer::PrescaleMode;

                frequency *= 2;
                uint16_t target_prescale = F_CPU / 256 / frequency;
                uint8_t mode;
                for (mode = 0; mode < timer.prescale_count; mode++) {
                    if (pgm_read_word(timer.prescale_values + mode) >= target_prescale) {
                        break;
                    }
                }

                uint8_t ocr = F_CPU / pgm_read_word(timer.prescale_values + mode) / frequency - 1;
                channel.set_output_compare(ocr);
                timer.set_prescale(static_cast<PrescaleMode>(mode+1));

                // Connect Pin to timer to toggle on each compare match.
                channel.set_mode(Timers::CompareOutputMode::Mode1);
            }

            inline void stop_tone() {
                typename Pin::TimerChannel channel;
                // Just disconnect the pin. We won't bother turning the timer off.
                channel.set_mode(Timers::CompareOutputMode::Mode0);
            }
    };
}

#endif //STRONG_IO_H