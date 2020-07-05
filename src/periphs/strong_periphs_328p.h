// Used to ensure that only one perpheral definition is imported.
// Modelled after the AVR IO headers.
#ifndef STRONG_PERIPHERALS_H
#  error "Include strong_periphs.h instead of this file."
#endif

#ifndef STRONG_PERIPHERALS_XXXX_H
#  define STRONG_PERIPHERALS_XXXX_H "328p"
#else
#  error "Attempted to include more than one strong_periphs_xxxx.h file"
#endif

#ifndef STRONG_PERIPHERALS_328P_H
#define STRONG_PERIPHERALS_328P_H

#include <avr/io.h>
#include <avr/pgmspace.h>
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
            static constexpr uint8_t prescale_count = TIMER_01_PRESCALE_COUNT;
            inline static constexpr const uint16_t* prescale_values() {
                return timer_01_prescale_values;
            }

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
            static constexpr uint8_t prescale_count = TIMER_01_PRESCALE_COUNT;
            inline static constexpr const uint16_t* prescale_values() {
                return timer_01_prescale_values;
            }

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
            static constexpr uint8_t prescale_count = TIMER_2_PRESCALE_COUNT;
            inline static constexpr const uint16_t* prescale_values() {
                return timer_2_prescale_values;
            }

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
            using ADCControl = IOReg<uint8_t, 0x7B>; // ADCSRB
            using CompControl = IOReg<uint8_t, 0x30 + __SFR_OFFSET>; // ACSR
            using DigitalInputDisable1 = IOReg<uint8_t, 0x7F>; // DIDR1

            inline static void disable_ain0_digital_input() {
                DigitalInputDisable1::set_bit(AIN0D);
            }

            inline static void enable_ain0_digital_input() {
                DigitalInputDisable1::clear_bit(AIN0D);
            }

            inline static void disable_ain1_digital_input() {
                DigitalInputDisable1::set_bit(AIN1D);
            }

            inline static void enable_ain1_digital_input() {
                DigitalInputDisable1::clear_bit(AIN1D);
            }

            inline static void disable_comparator() {
                CompControl::clear_bit(ACD);
            }

            inline static void enable_comparator() {
                CompControl::set_bit(ACD);
            }

            inline static void disable_bandgap_ref() {
                CompControl::clear_bit(ACBG);
            }

            inline static void enable_bandgap_ref() {
                CompControl::set_bit(ACBG);
            }

            inline static void disable_interrupt() {
                CompControl::clear_bit(ACIE);
            }

            inline static void enable_interrupt() {
                CompControl::set_bit(ACIE);
            }

            inline static void disable_input_capture() {
                CompControl::clear_bit(ACIC);
            }

            inline static void enable_input_capture() {
                CompControl::set_bit(ACIC);
            }

            inline static void set_interrupt_mode(ComparatorInterruptMode mode) {
                // Datasheet says we must disable the interrupt while changing these bits.
                uint8_t old_interrupt = CompControl::get_bit(ACIE);
                disable_interrupt();

                uint8_t int_mode = static_cast<uint8_t>(mode);
                int8_t mask = build_bitmask(ACIS1, ACIS0);
                CompControl::replace_bits(mask, int_mode);

                *CompControl::ptr() |= old_interrupt;
            }

            inline static bool is_positive_higher() {
                return CompControl::get_bit(ACO) != 0;
            }
    };

    class AnalogDigitalConverter {
        public:
            using MultiplexerSelection = IOReg<uint8_t, 0x7C>; // ADMUX
            using ControlA = IOReg<uint8_t, 0x7A>; // ADCSRA
            using ControlB = IOReg<uint8_t, 0x7B>; // ADCSRB
            using DataLow = IOReg<uint8_t, 0x78>; // ADCL
            using DataHigh = IOReg<uint8_t, 0x79>; // ADCH
            using DigitalInputDisable0 = IOReg<uint8_t, 0x7E>; // DIDR0

            inline static void set_voltage_ref(VoltageReference ref) {
                uint8_t vref = static_cast<uint8_t>(ref) << REFS0;
                uint8_t mask = build_bitmask(REFS1, REFS0);
                MultiplexerSelection::replace_bits(mask, vref);
            }

            inline void left_adjust() {
                MultiplexerSelection::set_bit(ADLAR);
            }

            inline static void right_adjust() {
                MultiplexerSelection::clear_bit(ADLAR);
            }

            inline static void set_channel(MultiplexerChannel channel) {
                uint8_t chan = static_cast<uint8_t>(channel);
                uint8_t mask = build_bitmask(MUX3, MUX2, MUX1, MUX0);
                MultiplexerSelection::replace_bits(mask, chan);
            }

            inline static void enable_adc() {
                ControlA::set_bit(ADEN);
            }

            inline static void disable_adc() {
                ControlA::clear_bit(ADEN);
            }

            inline static void start_conversion() {
                ControlA::set_bit(ADSC);
            }

            inline static void wait_for_conversion() {
                while (ControlA::get_bit(ADSC)) {}
            }

            inline static uint16_t read_data() {
                uint8_t low = DataLow::get_value();
                uint8_t high = DataHigh::get_value();

                return (high << 8) | low;
            }

            inline static void enable_auto_trigger() {
                ControlA::set_bit(ADATE);
            }

            inline static void disable_auto_trigger() {
                ControlA::clear_bit(ADATE);
            }

            inline static void enable_interrupt() {
                ControlA::set_bit(ADIE);
            }

            inline static void disable_interrupt() {
                ControlA::clear_bit(ADIE);
            }

            inline static void set_prescale(ADCPrescaler prescale) {
                uint8_t ps = static_cast<uint8_t>(prescale);
                uint8_t mask = build_bitmask(ADPS2, ADPS1, ADPS0);
                ControlA::replace_bits(mask, ps);
            }

            inline static void enable_comparator_multiplexer() {
                ControlB::set_bit(ACME);
            }

            inline static void disable_comparator_multiplexer() {
                ControlB::clear_bit(ACME);
            }

            inline static void set_auto_trigger(AutoTriggerSource source) {
                uint8_t src = static_cast<uint8_t>(source);
                uint8_t mask = build_bitmask(ADTS2, ADTS1, ADTS0);
                ControlB::replace_bits(mask, src);
            }

            template <typename Pin>
            inline static void disable_digital_input() {
                static_assert(is_analog_adc<Pin>::value, "Pin must be on the ADC");
                static_assert(is_digital<Pin>::value, "Pin must be on a digital port");
                DigitalInputDisable0::set_bit(Pin::analog_channel);
            }

            template <typename Pin>
            inline static void enable_digital_input() {
                static_assert(is_analog_adc<Pin>::value, "Pin must be on the ADC");
                static_assert(is_digital<Pin>::value, "Pin must be on a digital port");
                DigitalInputDisable0::clear_bit(Pin::analog_channel);
            }
    };
}

namespace Serials {
    enum class BitOrder {
        MSBFirst = 0b0,
        LSBFirst = 0b00100000,
    };

    enum class SPIClockPolarity {
        LeadingRising   = 0b0000,
        LeadingFalling  = 0b1000,
    };

    enum class SPIClockPhase {
        LeadingSample   = 0b000,
        LeadingSetup    = 0b100,
    };

    enum class SPIClockRate {
        CR4     = 0b00,
        CR16    = 0b01,
        CR64    = 0b10,
        CR128   = 0b11,
    };

    class SPI {
        public:
            using ControlRegister = IOReg<uint8_t, 0x2C + __SFR_OFFSET>; // SPCR0
            using StatusRegister = IOReg<uint8_t, 0x2D + __SFR_OFFSET>; // SPSR0
            using DataRegister = IOReg<uint8_t, 0x2E + __SFR_OFFSET>; // SPDR0

            inline static void enable_interrupt() {
                ControlRegister::set_bit(SPIE);
            }

            inline static void disable_interrupt() {
                ControlRegister::clear_bit(SPIE);
            }

            inline static void enable_spi() {
                ControlRegister::set_bit(SPE);
            }

            inline static void disable_spi() {
                ControlRegister::set_bit(SPE);
            }

            inline static void set_data_order(BitOrder order) {
                uint8_t val = static_cast<uint8_t>(order);
                uint8_t mask = build_bitmask(DORD);
                ControlRegister::replace_bits(mask, val);
            }

            inline static void set_master() {
                ControlRegister::set_bit(MSTR);
            }

            inline static void set_slave() {
                ControlRegister::clear_bit(MSTR);
            }

            inline static void set_clock_polarity(SPIClockPolarity pol) {
                uint8_t val = static_cast<uint8_t>(pol);
                uint8_t mask = build_bitmask(CPOL);
                ControlRegister::replace_bits(mask, val);
            }

            inline static void set_clock_phase(SPIClockPhase phase) {
                uint8_t val = static_cast<uint8_t>(phase);
                uint8_t mask = build_bitmask(CPHA);
                ControlRegister::replace_bits(mask, val);
            }

            inline static void set_clock_rate(SPIClockRate rate) {
                uint8_t val = static_cast<uint8_t>(rate);
                uint8_t mask = build_bitmask(SPR0, SPR1);
                ControlRegister::replace_bits(mask, val);
            }

            inline static void set_double_speed() {
                StatusRegister::set_bit(SPI2X);
            }

            inline static void set_normal_speed() {
                StatusRegister::clear_bit(SPI2X);
            }

            inline static void wait_for_finish() {
                // Apparently this nop can make it faster by causing the loop to
                // *not* run if the data is shifted fast enough.
                __asm__ volatile ("nop");
                while(!StatusRegister::get_bit(SPIF)) {}
            }
    };
}

#endif //STRONG_PERIPHERALS_328P_H
