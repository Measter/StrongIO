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
#include "common.h"

namespace Ports {
    class PortB {
        public:
            using InputRegister = IOReg<uint8_t, 0x03 + __SFR_OFFSET>; // PIN
            using ModeRegister = IOReg<uint8_t, 0x04 + __SFR_OFFSET>; // DDR
            using OutputRegister = IOReg<uint8_t, 0x05 + __SFR_OFFSET>; // PORT
    };

    class PortC {
        public:
            using InputRegister = IOReg<uint8_t, 0x06 + __SFR_OFFSET>; // PIN
            using ModeRegister = IOReg<uint8_t, 0x07 + __SFR_OFFSET>; // DDR
            using OutputRegister = IOReg<uint8_t, 0x08 + __SFR_OFFSET>; // PORT
    };

    class PortD {
        public:
            using InputRegister = IOReg<uint8_t, 0x09 + __SFR_OFFSET>; // PIN
            using ModeRegister = IOReg<uint8_t, 0x0A + __SFR_OFFSET>; // DDR
            using OutputRegister = IOReg<uint8_t, 0x0B + __SFR_OFFSET>; // PORT
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

            using ControlA = IOReg<uint8_t, 0x24 + __SFR_OFFSET>; // TCCR1A
            using ControlB = IOReg<uint8_t, 0x25 + __SFR_OFFSET>; // TCCR1B
            using InterruptMask = IOReg<uint8_t, 0x6E>; // TIMSK0
            using InterruptFlag = IOReg<uint8_t, 0x15 + __SFR_OFFSET>; // TIFR0
            using CounterValue = IOReg<CompareType, 0x26 + __SFR_OFFSET>; // TCNT0
            using GeneralTimerControl = IOReg<uint8_t, 0x23 + __SFR_OFFSET>; //GTCCR

            // Used in the loop for when searching for a prescale/output compare value
            // in tone output.
            const uint8_t prescale_count = TIMER_01_PRESCALE_COUNT;
            const uint16_t* prescale_values = timer_01_prescale_values;

            using OutputCompareA = IOReg<CompareType, 0x27 + __SFR_OFFSET>; //OCR0A
            static constexpr uint8_t channel_a_mode_bit0 = COM0A0;
            static constexpr uint8_t channel_a_mode_bit1 = COM0A1;
            static constexpr uint8_t channel_a_match_interrupt_bit = OCIE0A;

            using OutputCompareB = IOReg<CompareType, 0x28 + __SFR_OFFSET>; //OCR0A
            static constexpr uint8_t channel_b_mode_bit0 = COM0B0;
            static constexpr uint8_t channel_b_mode_bit1 = COM0B1;
            static constexpr uint8_t channel_b_match_interrupt_bit = OCIE0B;

            inline static void reset_timer_control() {
                ControlA::reset();
                ControlB::reset();
            }

            inline static void reset_counter() {
                CounterValue::reset();
            }

            inline static void enable_overflow_interrupt() {
                InterruptMask::set_bit(TOIE0);
            }

            inline static void disable_overflow_interrupt() {
                InterruptMask::set_bit(TOIE0);
            }

            inline static void set_prescale(PrescaleMode mode) {
                uint8_t mask = build_bitmask(CS00, CS01, CS02);
                ControlB::replace_bits(mask, static_cast<uint8_t>(mode));
            }

            inline static void set_waveform(WaveformMode mode) {
                uint8_t bits = static_cast<uint8_t>(mode);
                uint8_t mask = build_bitmask(WGM00, WGM01);
                ControlA::replace_bits(mask, bits & mask);
                mask = build_bitmask(WGM02);
                ControlB::replace_bits(mask, bits & mask);
            }
    };

    class Timer1 {
        public:
            using PrescaleMode = Timer01PrescaleMode;
            using WaveformMode = Timer1WaveformMode;
            using CompareType = uint16_t;

            using ControlA = IOReg<uint8_t, 0x80>; // TCCR1A
            using ControlB = IOReg<uint8_t, 0x81>; // TCCR1B
            using ControlC = IOReg<uint8_t, 0x82>; // TCCR1C
            using InterruptMask = IOReg<uint8_t, 0x6F>; // TIMSK1
            using InterruptFlag = IOReg<uint8_t, 0x16 + __SFR_OFFSET>; // TIFR1
            using CounterValue = IOReg<CompareType, 0x84>; // TCNT1
            using CounterValueLow = IOReg<uint8_t, 0x84>; // TCNT1L
            using CounterValueHigh = IOReg<uint8_t, 0x85>; // TCNT1H
            using InputCapture = IOReg<uint16_t, 0x86>; // ICR1
            using InputCaptureLow = IOReg<uint8_t, 0x86>; // ICR1L
            using InputCaptureHigh = IOReg<uint8_t, 0x87>; // ICR1H
            using GeneralTimerControl = IOReg<uint8_t, 0x23 + __SFR_OFFSET>; //GTCCR

            // Used in the loop for when searching for a prescale/output compare value
            // in tone output.
            const uint8_t prescale_count = TIMER_01_PRESCALE_COUNT;
            const uint16_t* prescale_values = timer_01_prescale_values;

            using OutputCompareA = IOReg<CompareType, 0x88>; // OCR1A
            using OutputCompareALow = IOReg<uint8_t, 0x88>; // OCR1AL
            using OutputCompareAHigh = IOReg<uint8_t, 0x89>; // OCR1AH
            static constexpr uint8_t channel_a_mode_bit0 = COM1A0;
            static constexpr uint8_t channel_a_mode_bit1 = COM1A1;
            static constexpr uint8_t channel_a_match_interrupt_bit = OCIE1A;

            using OutputCompareB = IOReg<CompareType, 0x8A>; // OCR1B
            using OutputCompareBLow = IOReg<uint8_t, 0x8A>; // OCR1BL
            using OutputCompareBHigh = IOReg<uint8_t, 0x8B>; // OCR1BH
            static constexpr uint8_t channel_b_mode_bit0 = COM1B0;
            static constexpr uint8_t channel_b_mode_bit1 = COM1B1;
            static constexpr uint8_t channel_b_match_interrupt_bit = OCIE1B;

            inline static void reset_timer_control() {
                ControlA::reset();
                ControlB::reset();
                ControlC::reset();
            }

            inline static void reset_counter() {
                CounterValue::reset();
            }

            inline static void enable_overflow_interrupt() {
                InterruptMask::set_bit(TOIE1);
            }

            inline static void disable_overflow_interrupt() {
                InterruptMask::clear_bit(TOIE1);
            }

            inline static void set_prescale(PrescaleMode mode) {
                uint8_t mask = build_bitmask(CS10, CS11, CS12);
                ControlB::replace_bits(mask, static_cast<uint8_t>(mode));
            }

            inline static void set_waveform(WaveformMode mode) {
                uint8_t bits = static_cast<uint8_t>(mode);
                uint8_t mask = build_bitmask(WGM11, WGM10);
                ControlA::replace_bits(mask, bits & mask);
                mask = build_bitmask(WGM13, WGM12);
                ControlB::replace_bits(mask, bits & mask);
            }
    };

    class Timer2 {
        public:
            using PrescaleMode = Timer2PrescaleMode;
            using WaveformMode = Timer02WaveformMode;
            using CompareType = uint8_t;

            using ControlA = IOReg<uint8_t, 0xB0>; // TCCR2A
            using ControlB = IOReg<uint8_t, 0xB1>; // TCCR2B
            using InterruptMask = IOReg<uint8_t, 0x70>; // TIMSK2
            using InterruptFlag = IOReg<uint8_t, 0x17 + __SFR_OFFSET>; // TIFR2
            using CounterValue = IOReg<CompareType, 0xB2>; // TCNT2
            using GeneralTimerControl = IOReg<uint8_t, 0x23 + __SFR_OFFSET>; //GTCCR
            using AsyncStatus = IOReg<uint8_t, 0xB6>; //ASSR

            // Used in the loop for when searching for a prescale/output compare value
            // in tone output.
            const uint8_t prescale_count = TIMER_2_PRESCALE_COUNT;
            const uint16_t* prescale_values = timer_2_prescale_values;

            using OutputCompareA = IOReg<CompareType, 0xB3>; // OCR2A
            static constexpr uint8_t channel_a_mode_bit0 = COM2A0;
            static constexpr uint8_t channel_a_mode_bit1 = COM2A1;
            static constexpr uint8_t channel_a_match_interrupt_bit = OCIE2A;

            using OutputCompareB = IOReg<CompareType, 0xB4>; // OCR2B
            static constexpr uint8_t channel_b_mode_bit0 = COM2B0;
            static constexpr uint8_t channel_b_mode_bit1 = COM2B1;
            static constexpr uint8_t channel_b_match_interrupt_bit = OCIE2B;

            inline static void reset_timer_control() {
                ControlA::reset();
                ControlB::reset();
            }

            inline static void reset_counter() {
                CounterValue::reset();
            }

            inline static void enable_overflow_interrupt() {
                InterruptMask::set_bit(TOIE2);
            }

            inline static void disable_overflow_interrupt() {
                InterruptMask::clear_bit(TOIE2);
            }

            inline static void set_prescale(PrescaleMode mode) {
                uint8_t mask = build_bitmask(CS22, CS21, CS20);
                ControlB::replace_bits(mask, static_cast<uint8_t>(mode));
            }

            inline static void set_waveform(WaveformMode mode) {
                uint8_t bits = static_cast<uint8_t>(mode);
                uint8_t mask = build_bitmask(WGM21, WGM20);
                ControlA::replace_bits(mask, bits & mask);
                mask = build_bitmask(WGM22);
                ControlB::replace_bits(mask, bits & mask);
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
            static constexpr uint8_t match_interrupt_bit = TimerType::channel_a_match_interrupt_bit;

            inline static constexpr void set_mode(CompareOutputMode mode) {
                uint8_t bits = static_cast<uint8_t>(mode) << mode_bit0;
                uint8_t mask = build_bitmask(mode_bit1, mode_bit0);
                TimerType::ControlA::replace_bits(mask, bits);
            }

            inline static constexpr void set_output_compare(typename TimerType::CompareType value) {
                TimerType::OutputCompareA::set_value(value);
            }

            inline static constexpr void enable_match_interrupt() {
                TimerType::InterruptMask::set_bit(match_interrupt_bit);
            }

            inline static constexpr void disable_match_interrupt() {
                TimerType::InterruptMask::clear_bit(match_interrupt_bit);
            }
    };

    template <class TimerType>
    class ChannelB {
        public:
            using Timer = TimerType;
            static constexpr uint8_t mode_bit0 = TimerType::channel_b_mode_bit0;
            static constexpr uint8_t mode_bit1 = TimerType::channel_b_mode_bit1;
            static constexpr uint8_t match_interrupt_bit = TimerType::channel_b_match_interrupt_bit;

            inline static constexpr void set_mode(CompareOutputMode mode) {
                uint8_t bits = static_cast<uint8_t>(mode) << mode_bit0;
                uint8_t mask = build_bitmask(mode_bit1, mode_bit0);
                TimerType::ControlA::replace_bits(mask, bits);
            }

            inline static constexpr void set_output_compare(typename TimerType::CompareType value) {
                TimerType::OutputCompareB::set_value(value);
            }

            inline static constexpr void enable_match_interrupt() {
                TimerType::InterruptMask::set_bit(match_interrupt_bit);
            }

            inline static constexpr void disable_match_interrupt() {
                TimerType::InterruptMask::clear_bit(match_interrupt_bit);
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

    enum class VoltageReference {
        AREF        = 0b00,
        AVCC        = 0b01,
        Internal1V1 = 0b11,
    };

    enum class MultiplexerChannel {
        ADC0         = 0b0000,
        ADC1         = 0b0001,
        ADC2         = 0b0010,
        ADC3         = 0b0011,
        ADC4         = 0b0100,
        ADC5         = 0b0101,
        ADC6         = 0b0110,
        ADC7         = 0b0111,
        TempSensor   = 0b1000,
        Interval1V1  = 0b1110,
        GND          = 0b1111,
    };

    enum class ADCPrescaler {
        PS2   = 0b001,
        PS4   = 0b010,
        PS8   = 0b011,
        PS16  = 0b100,
        PS32  = 0b101,
        PS64  = 0b110,
        PS128 = 0b111,
    };

    enum class AutoTriggerSource {
        FreeRunning         = 0b000,
        AnalogComparator    = 0b001,
        ExternalInt0        = 0b010,
        Timer0CompMatchA    = 0b011,
        Timer0Overflow      = 0b100,
        Timer1CompMatchB    = 0b101,
        Timer1Overflow      = 0b110,
        Timer1Capture       = 0b111,
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

    class AnalogDigitalConverter {
        public:
            volatile uint8_t* multiplexer_selection = &ADMUX;
            volatile uint8_t* control_register_a = &ADCSRA;
            volatile uint8_t* control_register_b = &ADCSRB;
            volatile uint8_t* data_register_low = &ADCL;
            volatile uint8_t* data_register_high = &ADCH;
            volatile uint8_t* digital_input_disable = &DIDR0;

            inline void set_voltage_ref(VoltageReference ref) {
                uint8_t vref = static_cast<uint8_t>(ref) << REFS0;
                *this->multiplexer_selection = (*this->multiplexer_selection & 0b00111111) | vref;
            }

            inline void left_adjust() {
                *this->multiplexer_selection |= (1<<ADLAR);
            }

            inline void right_adjust() {
                *this->multiplexer_selection &= ~(1<<ADLAR);
            }

            inline void set_channel(MultiplexerChannel channel) {
                uint8_t chan = static_cast<uint8_t>(channel);
                *this->multiplexer_selection = (*this->multiplexer_selection & 0b11110000) | chan;
            }

            inline void enable_adc() {
                *this->control_register_a |= (1<<ADEN);
            }

            inline void disable_adc() {
                *this->control_register_a &= ~(1<<ADEN);
            }

            inline void start_conversion() {
                *this->control_register_a |= (1<<ADSC);
            }

            inline void wait_for_conversion() {
                while(*this->control_register_a & (1 << ADSC)) {}
            }

            inline uint16_t read_data() {
                uint8_t low = *this->data_register_low;
                uint8_t high = *this->data_register_high;

                return (high << 8) | low;
            }

            inline void enable_auto_trigger() {
                *this->control_register_a |= (1<<ADATE);
            }

            inline void disable_auto_trigger() {
                *this->control_register_a &= ~(1<<ADATE);
            }

            inline void enable_interrupt() {
                *this->control_register_a |= (1<<ADIE);
            }

            inline void disable_interrupt() {
                *this->control_register_a &= ~(1<<ADIE);
            }

            inline void set_prescale(ADCPrescaler prescale) {
                uint8_t ps = static_cast<uint8_t>(prescale);
                *this->control_register_a = (*this->control_register_a & 0b11111000) | ps;
            }

            inline void enable_comparator_multiplexer() {
                *this->control_register_b |= (1<<ACME);
            }

            inline void disable_comparator_multiplexer() {
                *this->control_register_b &= ~(1<<ACME);
            }

            inline void set_auto_trigger(AutoTriggerSource source) {
                uint8_t src = static_cast<uint8_t>(source);
                *this->control_register_b = (*this->control_register_b & 0b11111000) | src;
            }

            template <typename Pin>
            inline void disable_digital_input() {
                static_assert(is_analog_adc<Pin>::value, "Pin must be on the ADC");
                static_assert(is_digital<Pin>::value, "Pin must be on a digital port");
                uint8_t bit = 1 << Pin::analog_channel;
                *this->digital_input_disable |= bit;
            }

            template <typename Pin>
            inline void enable_digital_input() {
                static_assert(is_analog_adc<Pin>::value, "Pin must be on the ADC");
                static_assert(is_digital<Pin>::value, "Pin must be on a digital port");
                uint8_t bit = 1 << Pin::analog_channel;
                *this->digital_input_disable &= ~bit;
            }
    };
}

namespace Pin {

    class D0 {
        public:
            using Port = Ports::PortD;
            static constexpr uint8_t digital_pin_bit = PD0;
            static constexpr uint8_t id = 0;
    };

    class D1 {
        public:
            using Port = Ports::PortD;
            static constexpr uint8_t digital_pin_bit = PD1;
            static constexpr uint8_t id = 1;
    };

    class D2 {
        public:
            using Port = Ports::PortD;
            static constexpr uint8_t digital_pin_bit = PD2;
            static constexpr uint8_t id = 2;
    };

    class D3 {
        public:
            using Port = Ports::PortD;
            using TimerChannel = Timers::ChannelB<Timers::Timer2>;
            static constexpr uint8_t digital_pin_bit = PD3;
            static constexpr uint8_t id = 3;
    };

    class D4 {
        public:
            using Port = Ports::PortD;
            static constexpr uint8_t digital_pin_bit = PD4;
            static constexpr uint8_t id = 4;
    };

    class D5 {
        public:
            using Port = Ports::PortD;
            using TimerChannel = Timers::ChannelB<Timers::Timer0>;
            static constexpr uint8_t digital_pin_bit = PD5;
            static constexpr uint8_t id = 5;
    };

    class D6 {
        public:
            using Port = Ports::PortD;
            using TimerChannel = Timers::ChannelA<Timers::Timer0>;
            using Tone = void;
            using AnalogCompPositive = Analog::AnalogComparator;
            static constexpr uint8_t digital_pin_bit = PD6;
            static constexpr uint8_t id = 6;
    };

    class D7 {
        public:
            using Port = Ports::PortD;
            using AnalogCompNegative = Analog::AnalogComparator;
            static constexpr uint8_t digital_pin_bit = PD7;
            static constexpr uint8_t id = 7;
    };

    class D8 {
        public:
            using Port = Ports::PortB;
            static constexpr uint8_t digital_pin_bit = PB0;
            static constexpr uint8_t id = 8;
    };


    class D9 {
        public:
            using Port = Ports::PortB;
            using TimerChannel = Timers::ChannelA<Timers::Timer1>;
            using Tone = void;
            static constexpr uint8_t digital_pin_bit = PB1;
            static constexpr uint8_t id = 9;
    };

    class D10 {
        public:
            using Port = Ports::PortB;
            using TimerChannel = Timers::ChannelB<Timers::Timer1>;
            static constexpr uint8_t digital_pin_bit = PB2;
            static constexpr uint8_t id = 10;
    };

    class D11 {
        public:
            using Port = Ports::PortB;
            using TimerChannel = Timers::ChannelA<Timers::Timer2>;
            using Tone = void;
            static constexpr uint8_t digital_pin_bit = PB3;
            static constexpr uint8_t id = 11;
    };

    class D12 {
        public:
            using Port = Ports::PortB;
            static constexpr uint8_t digital_pin_bit = PB4;
            static constexpr uint8_t id = 12;
    };

    class D13 {
        public:
            using Port = Ports::PortB;
            static constexpr uint8_t digital_pin_bit = PB5;
            static constexpr uint8_t id = 13;
    };

    using LedBuiltin = D13;

    class A0 {
        public:
            using AnalogConv = void;
            using Port = Ports::PortC;
            static constexpr uint8_t digital_pin_bit = PC0;
            static constexpr uint8_t id = 14;
            static constexpr Analog::MultiplexerChannel analog_channel = Analog::MultiplexerChannel::ADC0;
    };

    class A1 {
        public:
            using AnalogConv = void;
            using Port = Ports::PortC;
            static constexpr uint8_t digital_pin_bit = PC1;
            static constexpr uint8_t id = 15;
            static constexpr Analog::MultiplexerChannel analog_channel = Analog::MultiplexerChannel::ADC1;
    };

    class A2 {
        public:
            using AnalogConv = void;
            using Port = Ports::PortC;
            static constexpr uint8_t digital_pin_bit = PC2;
            static constexpr uint8_t id = 16;
            static constexpr Analog::MultiplexerChannel analog_channel = Analog::MultiplexerChannel::ADC2;
    };

    class A3 {
        public:
            using AnalogConv = void;
            using Port = Ports::PortC;
            static constexpr uint8_t digital_pin_bit = PC3;
            static constexpr uint8_t id = 17;
            static constexpr Analog::MultiplexerChannel analog_channel = Analog::MultiplexerChannel::ADC3;
    };

    class A4 {
        public:
            using AnalogConv = void;
            using Port = Ports::PortC;
            static constexpr uint8_t digital_pin_bit = PC4;
            static constexpr uint8_t id = 18;
            static constexpr Analog::MultiplexerChannel analog_channel = Analog::MultiplexerChannel::ADC4;
    };

    class A5 {
        public:
            using AnalogConv = void;
            using Port = Ports::PortC;
            static constexpr uint8_t digital_pin_bit = PC5;
            static constexpr uint8_t id = 19;
            static constexpr Analog::MultiplexerChannel analog_channel = Analog::MultiplexerChannel::ADC5;
    };

    class A6 {
        public:
            using AnalogConv = void;
            static constexpr uint8_t id = 20;
            static constexpr Analog::MultiplexerChannel analog_channel = Analog::MultiplexerChannel::ADC6;
    };

    class A7 {
        public:
            using AnalogConv = void;
            static constexpr uint8_t id = 21;
            static constexpr Analog::MultiplexerChannel analog_channel = Analog::MultiplexerChannel::ADC7;
    };
}

#endif //STRONG_PINS_328P_H
