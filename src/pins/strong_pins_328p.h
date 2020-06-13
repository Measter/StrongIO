// Used to ensure that only one pin definition is imported.
// Modelled after the AVR IO headers.
#ifndef STRONG_PINS_H
#  error "Include strong_pins.h instead of this file."
#endif

#ifndef STRONG_PINS_XXXX_H
#  define STRONG_PINS_XXXX_H "328p"
#else
#  error "Attempted to include more than one strong_pins_xxxx.h file"
#endif

#ifndef STRONG_PINS_328P_H
#define STRONG_PINS_328P_H

#include <avr/io.h>

namespace Ports {
    class PortB {
        public:
            volatile uint8_t* mode_register = &DDRB;
            volatile uint8_t* output_register = &PORTB;
            volatile uint8_t* input_register = &PINB;
    };

    class PortC {
        public:
            volatile uint8_t* mode_register = &DDRC;
            volatile uint8_t* output_register = &PORTC;
            volatile uint8_t* input_register = &PINC;
    };

    class PortD {
        public:
            volatile uint8_t* mode_register = &DDRD;
            volatile uint8_t* output_register = &PORTD;
            volatile uint8_t* input_register = &PIND;
    };
}

namespace Timers {
    #define TIMER_01_PRESCALE_COUNT 5
    const uint16_t PROGMEM timer_01_prescale_values[TIMER_01_PRESCALE_COUNT] = {1, 8, 64, 256, 1024};
    #define TIMER_2_PRESCALE_COUNT 7
    const uint16_t PROGMEM timer_2_prescale_values[TIMER_2_PRESCALE_COUNT] = {1, 8, 32, 64, 128, 256, 1024};

    // Note that the exact values of these variants are so they can just 
    // be cast, masked, and thrown into the appropriate register.

    enum class Timer01PrescaleMode {
        Stopped             = 0b000,
        PS1                 = 0b001,
        PS8                 = 0b010,
        PS64                = 0b011,
        PS256               = 0b100,
        PS1024              = 0b101,
        ExternalFalling     = 0b110,
        ExternalRising      = 0b111,
    };

    enum class Timer2PrescaleMode {
        Stopped             = 0b000,
        PS1                 = 0b001,
        PS8                 = 0b010,
        PS32                = 0b011,
        PS64                = 0b100,
        PS128               = 0b101,
        PS256               = 0b110,
        PS1024              = 0b111,
    };

    enum class Timer02WaveformMode {
        Normal                  = 0b00000,
        PWMPhaseCorrect_MAX     = 0b00001,
        CTC_OCRA                = 0b00010,
        PWMFast_MAX             = 0b00011,
        PWMPhaseCorrect_OCRA    = 0b01001,
        PWMFast_OCRA            = 0b01011,
    };

    enum class Timer1WaveformMode {
        Normal                  = 0b00000,
        PWMPhaseCorrect_MAX     = 0b00001, // 8-bit mode, named for consistancy with the other timers.
        PWM9bitPhaseCorrect     = 0b00010,
        PWM10bitPhaseCorrect    = 0b00011,
        CTC_OCRA                = 0b01000,
        PWMFast_MAX             = 0b01001, // 8-bit mode, named for consistancy with the other timers.
        PWM9bitFast             = 0b01010,
        PWM10bitFast            = 0b01011,
        PWMPhaseFreqCorrect_ICR  = 0b10000,
        PWMPhaseFreqCorrect_OCRA = 0b10001,
        PWMPhaseCorrect_ICR     = 0b10010,
        PWMPhaseCorrect_OCRA    = 0b10011,
        CTC_ICR                 = 0b11000,
        PWMFast_ICR             = 0b11010,
        PWMFast_OCRA            = 0b11011,
    };

    class Timer0 {
        public:
            using PrescaleMode = Timer01PrescaleMode;
            using WaveformMode = Timer02WaveformMode;
            using CompareType = uint8_t;
            volatile uint8_t* control_a = &TCCR0A;
            volatile uint8_t* control_b = &TCCR0B;
            volatile uint8_t* interrupt_mask = &TIMSK0;
            volatile uint8_t* interupt_flag = &TIFR0;
            volatile uint8_t* counter_value = &TCNT0;

            // Used in the loop for when searching for a prescale/output compare value
            // in tone output.
            const uint8_t prescale_count = TIMER_01_PRESCALE_COUNT;
            const uint16_t* prescale_values = timer_01_prescale_values;

            volatile uint8_t* output_compare_a = &OCR0A;
            static constexpr uint8_t channel_a_mode_bit0 = 1 << COM0A0;
            static constexpr uint8_t channel_a_mode_bit1 = 1 << COM0A1;
            static constexpr uint8_t channel_a_match_interrupt_bit = 1 << OCIE0A;

            volatile uint8_t* output_compare_b = &OCR0B;
            static constexpr uint8_t channel_b_mode_bit0 = 1 << COM0B0;
            static constexpr uint8_t channel_b_mode_bit1 = 1 << COM0B1;
            static constexpr uint8_t channel_b_match_interrupt_bit = 1 << OCIE0B;

            inline void reset_timer_control() {
                *this->control_a = 0;
                *this->control_b = 0;
            }

            inline void reset_counter() {
                *this->counter_value = 0;
            }

            inline void enable_overflow_interrupt() {
                *this->interrupt_mask |= 0b00000001;
            }

            inline void disable_overflow_interrupt() {
                *this->interrupt_mask &= 0b11111110;
            }

            inline void set_prescale(PrescaleMode mode) {
                *this->control_b = (*this->control_b & 0b11111000) | static_cast<uint8_t>(mode);
            }

            inline void set_waveform(WaveformMode mode) {
                uint8_t bits = static_cast<uint8_t>(mode);
                *this->control_a = (*this->control_a & 0b11111100) | (bits & 0b00000011);
                *this->control_b = (*this->control_b & 0b11100111) | (bits & 0b00001000);
            }
    };

    class Timer1 {
        public:
            using PrescaleMode = Timer01PrescaleMode;
            using WaveformMode = Timer1WaveformMode;
            using CompareType = uint16_t;
            volatile uint8_t* control_a = &TCCR1A;
            volatile uint8_t* control_b = &TCCR1B;
            volatile uint8_t* control_c = &TCCR1C;
            volatile uint8_t* interrupt_mask = &TIMSK1;
            volatile uint8_t* interupt_flag = &TIFR1;
            volatile uint8_t* counter_value = &TCNT1L;
            volatile uint8_t* counter_value_high = &TCNT1H;
            volatile uint8_t* input_capture = &ICR1L;
            volatile uint8_t* input_capture_high = &ICR1H;

            // Used in the loop for when searching for a prescale/output compare value
            // in tone output.
            const uint8_t prescale_count = TIMER_01_PRESCALE_COUNT;
            const uint16_t* prescale_values = timer_01_prescale_values;

            volatile uint16_t* output_compare_a = &OCR1A;
            volatile uint8_t* output_compare_a_low = &OCR1AL;
            volatile uint8_t* output_compare_a_high = &OCR1AH;
            static constexpr uint8_t channel_a_mode_bit0 = 1 << COM1A0;
            static constexpr uint8_t channel_a_mode_bit1 = 1 << COM1A1;
            static constexpr uint8_t channel_a_match_interrupt_bit = 1 << OCIE1A;

            volatile uint16_t* output_compare_b = &OCR1B;
            volatile uint8_t* output_compare_b_low = &OCR1BL;
            volatile uint8_t* output_compare_b_high = &OCR1BH;
            static constexpr uint8_t channel_b_mode_bit0 = 1 << COM1B0;
            static constexpr uint8_t channel_b_mode_bit1 = 1 << COM1B1;
            static constexpr uint8_t channel_b_match_interrupt_bit = 1 << OCIE1B;

            inline void reset_timer_control() {
                *this->control_a = 0;
                *this->control_b = 0;
                *this->control_c = 0;
            }

            inline void reset_counter() {
                *this->counter_value = 0;
            }

            inline void enable_overflow_interrupt() {
                *this->interrupt_mask |= 0b00000001;
            }

            inline void disable_overflow_interrupt() {
                *this->interrupt_mask &= 0b11111110;
            }

            inline void set_prescale(PrescaleMode mode) {
                *this->control_b = (*this->control_b & 0b11111000) | static_cast<uint8_t>(mode);
            }

            inline void set_waveform(WaveformMode mode) {
                uint8_t bits = static_cast<uint8_t>(mode);
                *this->control_a = (*this->control_a & 0b11111100) | (bits & 0b00000011);
                *this->control_b = (*this->control_b & 0b11100111) | (bits & 0b00011000);
            }
    };

    class Timer2 {
        public:
            using PrescaleMode = Timer2PrescaleMode;
            using WaveformMode = Timer02WaveformMode;
            using CompareType = uint8_t;
            volatile uint8_t* control_a = &TCCR2A;
            volatile uint8_t* control_b = &TCCR2B;
            volatile uint8_t* interrupt_mask = &TIMSK2;
            volatile uint8_t* interupt_flag = &TIFR2;
            volatile uint8_t* counter_value = &TCNT2;

            // Used in the loop for when searching for a prescale/output compare value
            // in tone output.
            const uint8_t prescale_count = TIMER_2_PRESCALE_COUNT;
            const uint16_t* prescale_values = timer_2_prescale_values;

            volatile uint8_t* output_compare_a = &OCR2A;
            static constexpr uint8_t channel_a_mode_bit0 = 1 << COM2A0;
            static constexpr uint8_t channel_a_mode_bit1 = 1 << COM2A1;
            static constexpr uint8_t channel_a_match_interrupt_bit = 1 << OCIE2A;

            volatile uint8_t* output_compare_b = &OCR2B;
            static constexpr uint8_t channel_b_mode_bit0 = 1 << COM2B0;
            static constexpr uint8_t channel_b_mode_bit1 = 1 << COM2B1;
            static constexpr uint8_t channel_b_match_interrupt_bit = 1 << OCIE2B;

            inline void reset_timer_control() {
                *this->control_a = 0;
                *this->control_b = 0;
            }

            inline void reset_counter() {
                *this->counter_value = 0;
            }

            inline void enable_overflow_interrupt() {
                *this->interrupt_mask |= 0b00000001;
            }

            inline void disable_overflow_interrupt() {
                *this->interrupt_mask &= 0b11111110;
            }

            inline void set_prescale(PrescaleMode mode) {
                *this->control_b = (*this->control_b & 0b11111000) | static_cast<uint8_t>(mode);
            }

            inline void set_waveform(WaveformMode mode) {
                uint8_t bits = static_cast<uint8_t>(mode);
                *this->control_a = (*this->control_a & 0b11111100) | (bits & 0b00000011);
                *this->control_b = (*this->control_b & 0b11100111) | (bits & 0b00001000);
            }
    };

    // These modes have these opaque names because what the channel mode does depends on the
    // waveform mode of the timer.
    enum class CompareOutputMode {
        Mode0 = 0b00,
        Mode1 = 0b01,
        Mode2 = 0b10,
        Mode3 = 0b11,
    };

    template <class TimerType>
    class ChannelA {
        public:
            using Timer = TimerType;
            static constexpr uint8_t mode_bit0 = TimerType::channel_a_mode_bit0;
            static constexpr uint8_t mode_bit1 = TimerType::channel_a_mode_bit1;

            inline void set_mode(CompareOutputMode mode) {
                uint8_t bits = static_cast<uint8_t>(mode) << 6;
                TimerType timer;
                *timer.control_a = (*timer.control_a & 0b00111111) | bits;
            }

            inline void set_output_compare(typename TimerType::CompareType value) {
                TimerType timer;
                *timer.output_compare_a = value;
            }

            inline void enable_match_interrupt() {
                TimerType timer;
                *timer.interrupt_mask |= TimerType::channel_a_match_interrupt_bit;
            }

            inline void disable_match_interrupt() {
                TimerType timer;
                *timer.interrupt_mask &= ~TimerType::channel_a_match_interrupt_bit;
            }
    };

    template <class TimerType>
    class ChannelB {
        public:
            using Timer = TimerType;
            static constexpr uint8_t mode_bit0 = TimerType::channel_b_mode_bit0;
            static constexpr uint8_t mode_bit1 = TimerType::channel_b_mode_bit1;

            inline void set_mode(CompareOutputMode mode) {
                uint8_t bits = static_cast<uint8_t>(mode) << 4;
                TimerType timer;
                *timer.control_a = (*timer.control_a & 0b00111111) | bits;
            }

            inline void set_output_compare(typename TimerType::CompareType value) {
                TimerType timer;
                *timer.output_compare_b = value;
            }

            inline void enable_match_interrupt() {
                TimerType timer;
                *timer.interrupt_mask |= TimerType::channel_b_match_interrupt_bit;
            }

            inline void disable_match_interrupt() {
                TimerType timer;
                *timer.interrupt_mask &= ~TimerType::channel_b_match_interrupt_bit;
            }
    };
}

namespace Analog {
    /// Specific values are so it can be cast to a uint8_t and put into register.
    enum class ComparatorInterruptMode {
        OutputToggle = 0b00,
        FallingEdge  = 0b10,
        RisingEdge   = 0b11,
    };

    class AnalogComparator {
        public:
            volatile uint8_t* adc_control_register = &ADCSRB;
            volatile uint8_t* comp_control_register = &ACSR;
            volatile uint8_t* digital_disable_register = &DIDR1;

            inline void disable_ain0_digital_input() {
                *this->digital_disable_register |= (1<<AIN0D);
            }

            inline void enable_ain0_digital_input() {
                *this->digital_disable_register &= ~(1<<AIN0D);
            }

            inline void disable_ain1_digital_input() {
                *this->digital_disable_register |= (1<<AIN1D);
            }

            inline void enable_ain1_digital_input() {
                *this->digital_disable_register &= ~(1<<AIN1D);
            }

            inline void disable_comparator() {
                *this->comp_control_register &= ~(1<<ACD);
            }

            inline void enable_comparator() {
                *this->comp_control_register |= (1<<ACD);
            }

            inline void disable_bandgap_ref() {
                *this->comp_control_register &= ~(1<<ACBG);
            }

            inline void enable_bandgap_ref() {
                *this->comp_control_register |= (1<<ACBG);
            }

            inline void disable_interrupt() {
                *this->comp_control_register &= ~(1<<ACIE);
            }

            inline void enable_interrupt() {
                *this->comp_control_register |= (1<<ACIE);
            }

            inline void disable_input_capture() {
                *this->comp_control_register &= ~(1<<ACIC);
            }

            inline void enable_input_capture() {
                *this->comp_control_register |= (1<<ACIC);
            }

            inline void set_interrupt_mode(ComparatorInterruptMode mode) {
                // Datasheet says we must disable the interrupt while changing these bits.
                uint8_t old_interrupt = *this->comp_control_register & (1<<ACIE);
                this->disable_interrupt();

                uint8_t int_mode = static_cast<uint8_t>(mode);
                *this->comp_control_register = (*this->comp_control_register & 0b11111100) | int_mode;

                *this->comp_control_register |= old_interrupt;
            }

            inline bool is_positive_higher() {
                return (*this->comp_control_register & (1<<ACO)) != 0;
            }
    };
}

namespace Pin {

    class D0 {
        public:
            using Port = Ports::PortD;
            static constexpr uint8_t digital_pin_bit = 1<<PD0;
            static constexpr uint8_t id = 0;
    };

    class D1 {
        public:
            using Port = Ports::PortD;
            static constexpr uint8_t digital_pin_bit = 1<<PD1;
            static constexpr uint8_t id = 1;
    };

    class D2 {
        public:
            using Port = Ports::PortD;
            static constexpr uint8_t digital_pin_bit = 1<<PD2;
            static constexpr uint8_t id = 2;
    };

    class D3 {
        public:
            using Port = Ports::PortD;
            using TimerChannel = Timers::ChannelB<Timers::Timer2>;
            static constexpr uint8_t digital_pin_bit = 1<<PD3;
            static constexpr uint8_t id = 3;
    };

    class D4 {
        public:
            using Port = Ports::PortD;
            static constexpr uint8_t digital_pin_bit = 1<<PD4;
            static constexpr uint8_t id = 4;
    };

    class D5 {
        public:
            using Port = Ports::PortD;
            using TimerChannel = Timers::ChannelB<Timers::Timer0>;
            static constexpr uint8_t digital_pin_bit = 1<<PD5;
            static constexpr uint8_t id = 5;
    };

    class D6 {
        public:
            using Port = Ports::PortD;
            using TimerChannel = Timers::ChannelA<Timers::Timer0>;
            using Tone = void;
            using AnalogCompPositive = Analog::AnalogComparator;
            static constexpr uint8_t digital_pin_bit = 1<<PD6;
            static constexpr uint8_t id = 6;
    };

    class D7 {
        public:
            using Port = Ports::PortD;
            using AnalogCompNegative = Analog::AnalogComparator;
            static constexpr uint8_t digital_pin_bit = 1<<PD7;
            static constexpr uint8_t id = 7;
    };

    class D8 {
        public:
            using Port = Ports::PortB;
            static constexpr uint8_t digital_pin_bit = 1<<PB0;
            static constexpr uint8_t id = 8;
    };


    class D9 {
        public:
            using Port = Ports::PortB;
            using TimerChannel = Timers::ChannelA<Timers::Timer1>;
            using Tone = void;
            static constexpr uint8_t digital_pin_bit = 1<<PB1;
            static constexpr uint8_t id = 9;
    };

    class D10 {
        public:
            using Port = Ports::PortB;
            using TimerChannel = Timers::ChannelB<Timers::Timer1>;
            static constexpr uint8_t digital_pin_bit = 1<<PB2;
            static constexpr uint8_t id = 10;
    };

    class D11 {
        public:
            using Port = Ports::PortB;
            using TimerChannel = Timers::ChannelA<Timers::Timer2>;
            using Tone = void;
            static constexpr uint8_t digital_pin_bit = 1<<PB3;
            static constexpr uint8_t id = 11;
    };

    class D12 {
        public:
            using Port = Ports::PortB;
            static constexpr uint8_t digital_pin_bit = 1<<PB4;
            static constexpr uint8_t id = 12;
    };

    class D13 {
        public:
            using Port = Ports::PortB;
            static constexpr uint8_t digital_pin_bit = 1<<PB5;
            static constexpr uint8_t id = 13;
    };

    using LedBuiltin = D13;

    class A0 {
        public:
            using AnalogConv = void;
            using Port = Ports::PortC;
            static constexpr uint8_t digital_pin_bit = 1<<PC0;
            static constexpr uint8_t id = 14;
            static constexpr uint8_t analog_channel = 0;
    };

    class A1 {
        public:
            using AnalogConv = void;
            using Port = Ports::PortC;
            static constexpr uint8_t digital_pin_bit = 1<<PC1;
            static constexpr uint8_t id = 15;
            static constexpr uint8_t analog_channel = 1;
    };

    class A2 {
        public:
            using AnalogConv = void;
            using Port = Ports::PortC;
            static constexpr uint8_t digital_pin_bit = 1<<PC2;
            static constexpr uint8_t id = 16;
            static constexpr uint8_t analog_channel = 2;
    };

    class A3 {
        public:
            using AnalogConv = void;
            using Port = Ports::PortC;
            static constexpr uint8_t digital_pin_bit = 1<<PC3;
            static constexpr uint8_t id = 17;
            static constexpr uint8_t analog_channel = 3;
    };

    class A4 {
        public:
            using AnalogConv = void;
            using Port = Ports::PortC;
            static constexpr uint8_t digital_pin_bit = 1<<PC4;
            static constexpr uint8_t id = 18;
            static constexpr uint8_t analog_channel = 4;
    };

    class A5 {
        public:
            using AnalogConv = void;
            using Port = Ports::PortC;
            static constexpr uint8_t digital_pin_bit = 1<<PC5;
            static constexpr uint8_t id = 19;
            static constexpr uint8_t analog_channel = 5;
    };

    class A6 {
        public:
            using AnalogConv = void;
            static constexpr uint8_t id = 20;
            static constexpr uint8_t analog_channel = 6;
    };

    class A7 {
        public:
            using AnalogConv = void;
            static constexpr uint8_t id = 21;
            static constexpr uint8_t analog_channel = 7;
    };
}

#endif //STRONG_PINS_328P_H
