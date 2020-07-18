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

namespace Peripherals {

    namespace MCU {
        enum class SleepMode {
            Idle                = 0b0000,
            ADCNoiseReduciton   = 0b0010,
            PowerDown           = 0b0100,
            PowerSave           = 0b0110,
            Standby             = 0b1100,
            ExtendedStandby     = 0b1110,
        };

        class Power {
            public:
                using SleepControl = IOReg<uint8_t, 0x33 + __SFR_OFFSET>; // SMCR
                using GeneralControl = IOReg<uint8_t, 0x35 + __SFR_OFFSET>; //MCUCR
                using PowerReduction = IOReg<uint8_t, 0x64>; // PRR

                inline static void disable_twi() {
                    PowerReduction::set_bit(PRTWI);
                }

                inline static void enable_twi() {
                    PowerReduction::clear_bit(PRTWI);
                }

                inline static void disable_timer0() {
                    PowerReduction::set_bit(PRTIM0);
                }

                inline static void enable_timer0() {
                    PowerReduction::clear_bit(PRTIM0);
                }

                inline static void disable_timer1() {
                    PowerReduction::set_bit(PRTIM1);
                }

                inline static void enable_timer1() {
                    PowerReduction::clear_bit(PRTIM1);
                }

                inline static void disable_timer2() {
                    PowerReduction::set_bit(PRTIM2);
                }

                inline static void enable_timer2() {
                    PowerReduction::clear_bit(PRTIM2);
                }

                inline static void disable_spi() {
                    PowerReduction::set_bit(PRSPI);
                }

                inline static void enable_spi() {
                    PowerReduction::clear_bit(PRSPI);
                }
                
                inline static void disable_usart() {
                    PowerReduction::set_bit(PRUSART0);
                }

                inline static void enable_usart() {
                    PowerReduction::clear_bit(PRUSART0);
                }

                inline static void disable_adc() {
                    PowerReduction::set_bit(PRADC);
                }

                inline static void enable_adc() {
                    PowerReduction::clear_bit(PRADC);
                }

                inline static void set_sleep_mode(SleepMode mode) {
                    uint8_t val = static_cast<uint8_t>(mode);
                    uint8_t mask = build_bitmask(SM2, SM1, SM0);
                    SleepControl::replace_bits(mask, val);
                }

                inline static void enable_sleep_bit() {
                    SleepControl::set_bit(SE);
                }

                inline static void disable_sleep_bit() {
                    SleepControl::clear_bit(SE);
                }

                inline static void sleep() {
                    enable_sleep_bit();
                    __asm__ volatile ("sleep");
                    disable_sleep_bit();
                }

                inline static void move_interrupt_table_to_flash_start() {
                    // Both writes need to be done within 4 cycles.
                    uint8_t val = GeneralControl::get_value();
                    GeneralControl::set_value(val | (1 << IVCE));
                    GeneralControl::set_value(val & ~(1 << IVSEL));
                }

                inline static void move_interrupt_table_to_boot_flash() {
                    // Both writes need to be done within 4 cycles.
                    uint8_t val = GeneralControl::get_value();
                    GeneralControl::set_value(val | (1 << IVCE));
                    GeneralControl::set_value(val | (1 << IVSEL));
                }

                inline static void enable_pullups() {
                    GeneralControl::clear_bit(PUD);
                }

                inline static void disable_pullups() {
                GeneralControl::set_bit(PUD);
            }
        };
    }
    
    namespace Interrupts {
        enum class ExternalInterruptMode {
            Low     = 0b00,
            Change  = 0b01,
            Falling = 0b10,
            Rising  = 0b11,
        };

        class ExternalInterrupts {
            public:
                using ExternalControlRegisterA = IOReg<uint8_t, 0x69>; // EICRA
                using ExternalMaskRegister = IOReg<uint8_t, 0x1D + __SFR_OFFSET>; // EIMSK
                using ExternalFlagRegister = IOReg<uint8_t, 0x1C + __SFR_OFFSET>; // EIFR

                using ChangeControlRegister = IOReg<uint8_t, 0x68>; // PCICR
                using ChangeFlagRegister = IOReg<uint8_t, 0x1B + __SFR_OFFSET>; // PCIFR
                using ChangeMaskRegister0 = IOReg<uint8_t, 0x6B>; // PCMSK0
                using ChangeMaskRegister1 = IOReg<uint8_t, 0x6C>; // PCMSK1
                using ChangeMaskRegister2 = IOReg<uint8_t, 0x6D>; // PCMSK2

                inline static void enable_change_0() {
                    ChangeControlRegister::set_bit(PCIE0);
                }

                inline static void disable_change_0() {
                    ChangeControlRegister::clear_bit(PCIE0);
                }

                inline static void enable_change_1() {
                    ChangeControlRegister::set_bit(PCIE1);
                }

                inline static void disable_change_1() {
                    ChangeControlRegister::clear_bit(PCIE1);
                }

                inline static void enable_change_2() {
                    ChangeControlRegister::set_bit(PCIE2);
                }

                inline static void disable_change_2() {
                    ChangeControlRegister::clear_bit(PCIE2);
                }

                inline static void set_int0_sense_mode(ExternalInterruptMode mode) {
                    uint8_t val = static_cast<uint8_t>(mode);
                    uint8_t mask = build_bitmask(ISC00, ISC01);
                    ExternalControlRegisterA::replace_bits(mask, val);
                }

                inline static void set_int1_sense_mode(ExternalInterruptMode mode) {
                    uint8_t val = static_cast<uint8_t>(mode) << 2;
                    uint8_t mask = build_bitmask(ISC10, ISC11);
                    ExternalControlRegisterA::replace_bits(mask, val);
                }

                inline static void enable_int0() {
                    ExternalMaskRegister::set_bit(INT0);
                }

                inline static void disable_int0() {
                    ExternalMaskRegister::clear_bit(INT0);
                }

                inline static void enable_int1() {
                    ExternalMaskRegister::set_bit(INT1);
                }

                inline static void disable_int1() {
                ExternalMaskRegister::clear_bit(INT1);
            }
        };
    }

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

                using ControlA = IOReg<uint8_t, 0x24 + __SFR_OFFSET>; // TCCR0A
                using ControlB = IOReg<uint8_t, 0x25 + __SFR_OFFSET>; // TCCR0B
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
                    InterruptMask::clear_bit(TOIE0);
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

                // This is needed for the Tone type. It's done here because it's needed
                // on each timer, and we want to avoid a progmem read if we can.
                inline static PrescaleMode next_bigger_prescale(uint16_t min) {
                    if (min < 1)
                        return PrescaleMode::PS1;
                    else if (min < 8)
                        return PrescaleMode::PS8;
                    else if (min < 64)
                        return PrescaleMode::PS64;
                    else if (min < 256)
                        return PrescaleMode::PS256;
                    return PrescaleMode::PS1024;
                }

                inline static uint16_t get_prescale_value(PrescaleMode mode) {
                    switch (mode) {
                        case PrescaleMode::PS1: return 1;
                        case PrescaleMode::PS8: return 8;
                        case PrescaleMode::PS64: return 64;
                        case PrescaleMode::PS256: return 256;
                        case PrescaleMode::PS1024: return 1024;
                        default: return 0;
                    }
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

                // This is needed for the Tone type. It's done here because it's needed
                // on each timer, and we want to avoid a progmem read if we can.
                inline static PrescaleMode next_bigger_prescale(uint16_t min) {
                    if (min < 1)
                        return PrescaleMode::PS1;
                    else if (min < 8)
                        return PrescaleMode::PS8;
                    else if (min < 64)
                        return PrescaleMode::PS64;
                    else if (min < 256)
                        return PrescaleMode::PS256;
                    return PrescaleMode::PS1024;
                }

                inline static uint16_t get_prescale_value(PrescaleMode mode) {
                    switch (mode) {
                        case PrescaleMode::PS1: return 1;
                        case PrescaleMode::PS8: return 8;
                        case PrescaleMode::PS64: return 64;
                        case PrescaleMode::PS256: return 256;
                        case PrescaleMode::PS1024: return 1024;
                        default: return 0;
                    }
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

                // This is needed for the Tone type. It's done here because it's needed
                // on each timer, and we want to avoid a progmem read if we can.
                inline static PrescaleMode next_bigger_prescale(uint16_t min) {
                    if (min < 1)
                        return PrescaleMode::PS1;
                    else if (min < 8)
                        return PrescaleMode::PS8;
                    else if (min < 32)
                        return PrescaleMode::PS32;
                    else if (min < 64)
                        return PrescaleMode::PS64;
                    else if (min < 128)
                        return PrescaleMode::PS128;
                    else if (min < 256)
                        return PrescaleMode::PS256;
                    return PrescaleMode::PS1024;
                }

                inline static uint16_t get_prescale_value(PrescaleMode mode) {
                    switch (mode) {
                        case PrescaleMode::PS1: return 1;
                        case PrescaleMode::PS8: return 8;
                        case PrescaleMode::PS32: return 32;
                        case PrescaleMode::PS64: return 64;
                        case PrescaleMode::PS128: return 128;
                        case PrescaleMode::PS256: return 256;
                        case PrescaleMode::PS1024: return 1024;
                        default: return 0;
                    }
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
        enum class SPIBitOrder {
            MSBFirst,
            LSBFirst,
        };

        enum class ClockPolarity {
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

                inline static void set_data_order(SPIBitOrder order) {
                    if(order == SPIBitOrder::LSBFirst) {
                        ControlRegister::set_bit(DORD);
                    } else {
                        ControlRegister::clear_bit(DORD);
                    }
                }

                inline static void enable_master() {
                    ControlRegister::set_bit(MSTR);
                }

                inline static void disable_slave() {
                    ControlRegister::clear_bit(MSTR);
                }

                inline static void set_clock_polarity(ClockPolarity pol) {
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

                inline static void enable_double_speed() {
                    StatusRegister::set_bit(SPI2X);
                }

                inline static void disable_double_speed() {
                    StatusRegister::clear_bit(SPI2X);
                }

                inline static void wait_for_finish() {
                    // Apparently this nop can make it faster by causing the loop to
                    // *not* run if the data is shifted fast enough.
                    __asm__ volatile ("nop");
                    while(!StatusRegister::get_bit(SPIF)) {}
                }
        };

        enum class USARTMode {
            Async       = 0b00000000,
            Sync        = 0b01000000,
            MasterSPI   = 0b11000000,
        };

        enum class USARTParityMode {
            Disabled    = 0b000000,
            Even        = 0b100000,
            Odd         = 0b110000,
        };

        enum class USARTStopBit {
            Bits1,
            Bits2,
        };

        enum class USARTCharSize {
            Bits5 = 0b0000,
            Bits6 = 0b0010,
            Bits7 = 0b0100,
            Bits8 = 0b0110,
            Bits9 = 0b1110,
        };
        
        // I've separated the register definition from the configuration because the 2560
        // has multiple USARTs, which are all identically configured.
        class USART0 {
            public:
                using DataRegister = IOReg<uint8_t, 0xC6>; // UDR0
                using ControlStatusRegisterA = IOReg<uint8_t, 0xC0, 0x20>; // UCSR0A
                using ControlStatusRegisterB = IOReg<uint8_t, 0xC1>; // UCSR0B
                using ControlStatusRegisterC = IOReg<uint8_t, 0xC2, 0x06>; // UCSR0C

                using BaudRateRegister = IOReg<uint16_t, 0xC4>; // UBRR0
                using BaudRateRegisterLow = IOReg<uint8_t, 0xC4>; // UBRR0L
                using BaudRateRegisterHigh = IOReg<uint8_t, 0xC5>; // UBRR0H
        };

        template <typename USART>
        class USARTConfig {
            public:
                /// Control Register A

                inline static bool is_receive_complete() {
                    return USART::ControlStatusRegisterA::get_bit(RXC0) != 0;
                }

                inline static void wait_for_receive() {
                    while (!is_receive_complete()) {}
                }

                inline static bool is_transmit_complete() {
                    return USART::ControlStatusRegisterA::get_bit(TXC0) != 0;
                }

                inline static void wait_for_transmit() {
                    while (!is_transmit_complete()) {}
                }

                inline static void clear_transmit_complete() {
                    // datasheet says we should always set FE0, DOR0, and UPE0 to 0 when writing.
                    uint8_t mask = build_bitmask(FE0, DOR0, UPE0, TXC0);
                    uint8_t val = build_bitmask(TXC0);
                    USART::ControlStatusRegisterA::replace_bits(mask, val);
                }

                inline static bool is_data_register_empty() {
                    return USART::ControlStatusRegisterA::get_bit(UDRE0) != 0;
                }

                inline static void wait_for_data_register_empty() {
                    while (!is_data_register_empty()) {}
                }

                // Set this to 0 when writing to Control A.
                inline static bool had_frame_error() {
                    return USART::ControlStatusRegisterA::get_bit(FE0) != 0;
                }

                //Set this to 0 when writing to Control A.
                inline static bool had_data_overrun() {
                    return USART::ControlStatusRegisterA::get_bit(DOR0) != 0;
                }

                // Set this to 0 when writing to Control A.
                inline static bool had_parity_error() {
                    return USART::ControlStatusRegisterA::get_bit(UPE0) != 0;
                }

                inline static void enable_double_transmit_speed() {
                    // datasheet says we should alawys set FE0, DOR0, and UPE0 to 0 when writing.
                    uint8_t mask = build_bitmask(FE0, DOR0, UPE0, U2X0);
                    uint8_t val = build_bitmask(U2X0);
                    USART::ControlStatusRegisterA::replace_bits(mask, val);
                }

                inline static void disable_double_transmit_speed() {
                    // datasheet says we should alawys set FE0, DOR0, and UPE0 to 0 when writing.
                    uint8_t mask = build_bitmask(FE0, DOR0, UPE0, U2X0);
                    USART::ControlStatusRegisterA::replace_bits(mask, 0);
                }

                inline static void enable_multi_proc() {
                    // datasheet says we should alawys set FE0, DOR0, and UPE0 to 0 when writing.
                    uint8_t mask = build_bitmask(FE0, DOR0, UPE0, MPCM0);
                    uint8_t val = build_bitmask(MPCM0);
                    USART::ControlStatusRegisterA::replace_bits(mask, val);
                }

                inline static void disable_multi_proc() {
                    // datasheet says we should alawys set FE0, DOR0, and UPE0 to 0 when writing.
                    uint8_t mask = build_bitmask(FE0, DOR0, UPE0, MPCM0);
                    USART::ControlStatusRegisterA::replace_bits(mask, 0);
                }
        
                
                /// Control Register B

                inline static void enable_receive_interrupt() {
                    USART::ControlStatusRegisterB::set_bit(RXCIE0);
                }

                inline static void disable_receive_interrupt() {
                    USART::ControlStatusRegisterB::clear_bit(RXCIE0);
                }
                
                inline static void enable_transmit_interrupt() {
                    USART::ControlStatusRegisterB::set_bit(TXCIE0);
                }

                inline static void disable_transmit_interrupt() {
                    USART::ControlStatusRegisterB::clear_bit(TXCIE0);
                }

                inline static void enable_data_empty_interrupt() {
                    USART::ControlStatusRegisterB::set_bit(UDRIE0);
                }

                inline static void disable_data_empty_interrupt() {
                    USART::ControlStatusRegisterB::clear_bit(UDRIE0);
                }

                inline static void enable_receiver() {
                    USART::ControlStatusRegisterB::set_bit(RXEN0);
                }

                inline static void disable_receiver() {
                    USART::ControlStatusRegisterB::clear_bit(RXEN0);
                }

                inline static void enable_transmit() {
                    USART::ControlStatusRegisterB::set_bit(TXEN0);
                }

                inline static void disable_transmit() {
                    USART::ControlStatusRegisterB::clear_bit(TXEN0);
                }

                /// Control Register C

                inline static void set_mode(USARTMode mode) {
                    uint8_t val = static_cast<uint8_t>(mode);
                    uint8_t mask = build_bitmask(UMSEL00, UMSEL01);
                    USART::ControlStatusRegisterC::replace_bits(mask, val & mask);
                }

                inline static void set_parity(USARTParityMode mode) {
                    uint8_t val = static_cast<uint8_t>(mode);
                    uint8_t mask = build_bitmask(UPM00, UPM01);
                    USART::ControlStatusRegisterC::replace_bits(mask, val & mask);
                }

                inline static void set_stop_bit(USARTStopBit mode) {
                    if (mode == USARTStopBit::Bits1) {
                        USART::ControlStatusRegisterC::clear_bit(USBS0);
                    } else {
                        USART::ControlStatusRegisterC::set_bit(USBS0);
                    }
                }

                inline static void set_usart_character_size(USARTCharSize size) {
                    uint8_t val = static_cast<uint8_t>(size);
                    uint8_t mask = build_bitmask(UCSZ00, UCSZ01);
                    USART::ControlStatusRegisterC::replace_bits(mask, val & mask);
                    
                    // These don't get to use the SBI and CBI instructions.
                    mask = build_bitmask(UCSZ02);
                    USART::ControlStatusRegisterB::replace_bits(mask, (val >> 1) & mask);
                }

                inline static void set_spi_data_order(SPIBitOrder order) {
                    if(order == SPIBitOrder::LSBFirst) {
                        USART::ControlStatusRegisterC::set_bit(UDORD0);
                    } else {
                        USART::ControlStatusRegisterC::clear_bit(UDORD0);
                    }
                }

                inline static void set_spi_clock_phase(SPIClockPhase phase) {
                    if(phase == SPIClockPhase::LeadingSample) {
                        USART::ControlStatusRegisterC::clear_bit(UCPHA0);
                    } else {
                        USART::ControlStatusRegisterC::set_bit(UCPHA0);
                    }
                }

                inline static void set_clock_polarity(ClockPolarity pol) {
                    if(pol == ClockPolarity::LeadingRising) {
                        USART::ControlStatusRegisterC::clear_bit(UCPOL0);
                    } else {
                        USART::ControlStatusRegisterC::set_bit(UCPOL0);
                    }
                }

                /// Other

                inline static uint16_t read_data_9b() {
                    uint16_t ninth = USART::ControlStatusRegisterB::get_bit(RXB80) << 7;
                    return ninth | USART::DataRegister::get_value();
                }

                inline static void write_data_9b(uint16_t data) {
                    uint8_t mask = build_bitmask(TXB80);
                    USART::ControlStatusRegisterB::replace_bits(mask, data >> 8);
                    USART::DataRegister::set_value(data);
                }

                inline static void set_baud_rate(uint16_t rate) {
                    USART::BaudRateRegister::set_value(rate & 0xFFF);
                }
        };

        enum class TWIPrescale {
            PS1  = 0b00,
            PS4  = 0b01,
            PS16 = 0b10,
            PS64 = 0b11,
        };

        enum class TWIStatus {
            Mst_StartTxd                 = 0x08,
            Mst_StartRepeated            = 0x10,
            Mst_SLAWTx_ACK               = 0x18,
            Mst_SLAWTx_NoACK             = 0x20,
            Mst_DataTx_ACK               = 0x28,
            Mst_DataTx_NoACK             = 0x30,
            Mst_ArbitLost                = 0x38,
            Mst_SLARTx_ACK               = 0x40,
            Mst_SLARTx_NoACK             = 0x48,
            Mst_DataRcv_ACK              = 0x50,
            Mst_DataRcv_NoACK            = 0x58,

            Slv_SLARRcv_Ack               = 0xA8,
            Slv_TxArbitLost_OwnSLARW_ACK  = 0xB0,
            Slv_DataTx_ACK                = 0xB8,
            Slv_DataTx_NoACK              = 0xC0,
            Slv_LastDataTx_ACK            = 0xC8,
            Slv_SLAWRcv_ACK               = 0x60,
            Slv_RcvArbitLost_OwnSLARW_ACK = 0x68,
            Slv_GenCall_ACK               = 0x70,
            Slv_ArbitLost_GenCall_ACK     = 0x78,
            Slv_PrevAddrSLAW_DataRcv_ACK  = 0x80,
            Slv_PrevAddrSLAW_DataRcv_NoACK = 0x88,
            Slv_PrevAddrGenCall_DataRcv_ACK = 0x90,
            Slv_PrevAddrGenCall_DataRcv_NoACK = 0x98,
            Slv_StopRptStart              = 0xA0,

            NoState                       = 0xF8,
            BusError                      = 0x00
        };

        class TWI {
            public:
                using BitRateRegister = IOReg<uint8_t, 0xB8>; // TWBR
                using StatusRegister = IOReg<uint8_t, 0xB9, 0xF8>; // TWSR
                using SlaveAddress = IOReg<uint8_t, 0xBA, 0xFE>; // TWAR
                using DataRegister = IOReg<uint8_t, 0xBB, 0xFF>; // TWDR
                using ControlRegister = IOReg<uint8_t, 0xBC>; // TWCR
                using SlaveAddressMask = IOReg<uint8_t, 0xBD>; // TWAMR

                inline static void set_prescaler(TWIPrescale pres) {
                    uint8_t val = static_cast<uint8_t>(pres);
                    uint8_t mask = build_bitmask(TWPS0, TWPS1);
                    StatusRegister::replace_bits(mask, val);
                }

                inline static TWIStatus get_status() {
                    uint8_t mask = build_bitmask(TWS7, TWS6, TWS5, TWS4, TWS3);
                    uint8_t bits = StatusRegister::get_value() & mask;
                    return static_cast<TWIStatus>(bits);
                }

                // Note: Shifts the address left by 1 before saving.
                inline static void set_slave_address(uint8_t addr) {
                    SlaveAddress::replace_bits(0xFE, addr << 1);
                }

                // Note: Shifts the address right by one before returning.
                inline static uint8_t get_slave_address() {
                    return SlaveAddress::get_value() >> 1;
                }

                // Note: Shifts mask left by 1 before saving.
                inline static void set_slave_address_mask(uint8_t mask) {
                    SlaveAddressMask::replace_bits(0xFE, mask << 1);
                }

                // Nate: Shifts mask right be 1 before returning.
                inline static uint8_t get_slave_address_mask() {
                    return SlaveAddressMask::get_value() >> 1;
                }

                inline static void enable_general_call() {
                    SlaveAddress::set_bit(TWGCE);
                }

                inline static void disable_general_call() {
                    SlaveAddress::clear_bit(TWGCE);
                }

                inline static bool get_interrupt_flag() {
                    return ControlRegister::get_bit(TWINT) != 0;
                }

                inline static void clear_interrupt_flag() {
                    ControlRegister::set_bit(TWINT);
                }

                inline static void wait_for_interrupt_flag() {
                    while(!get_interrupt_flag()) {}
                }

                inline static void enable_ack() {
                    ControlRegister::set_bit(TWEA);
                }

                inline static void virtual_disconnect() {
                    ControlRegister::clear_bit(TWEA);
                }

                inline static void set_start() {
                    ControlRegister::set_bit(TWSTA);
                }

                inline static void clear_start() {
                    ControlRegister::clear_bit(TWSTA);
                }

                inline static void set_stop() {
                    ControlRegister::set_bit(TWSTO);
                }

                inline static void clear_slave_error() {
                    ControlRegister::clear_bit(TWSTO);
                }

                inline static bool get_write_collision_flag() {
                    return ControlRegister::get_bit(TWWC) != 0;
                }

                inline static void enable_twi() {
                    ControlRegister::set_bit(TWEN);
                }

                inline static void disable_twi() {
                    ControlRegister::clear_bit(TWEN);
                }

                inline static void enable_interrupt() {
                    ControlRegister::set_bit(TWIE);
                }

                inline static void disable_interrupt() {
                    ControlRegister::clear_bit(TWIE);
                }
        };
    }
    
}

#endif //STRONG_PERIPHERALS_328P_H
