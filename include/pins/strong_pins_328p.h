// Used to ensure that only one pin definition is imported.
// Modelled after the AVR IO headers.
#ifndef STRONG_PINS_H
    #error "Include strong_pins.h instead of this file."
#endif

#ifndef STRONG_PINS_XXXX_H
    #define STRONG_PINS_XXXX_H "328p"
#else
    #error "Attempted to include more than one strong_pins_xxxx.h file"
#endif

#ifndef STRONG_PINS_328P_H
    #define STRONG_PINS_328P_H

    #include "strong_periphs.h"
    #include <avr/io.h>

namespace Pin {
    class D0 {
      public:
        using Port                               = Peripherals::Ports::PortD;
        static constexpr uint8_t digital_pin_bit = PD0;
        static constexpr uint8_t id              = 0;
    };

    class D1 {
      public:
        using Port                               = Peripherals::Ports::PortD;
        static constexpr uint8_t digital_pin_bit = PD1;
        static constexpr uint8_t id              = 1;
    };

    class D2 {
      public:
        using Port                               = Peripherals::Ports::PortD;
        static constexpr uint8_t digital_pin_bit = PD2;
        static constexpr uint8_t id              = 2;
    };

    class D3 {
      public:
        using Port                               = Peripherals::Ports::PortD;
        using TimerChannel                       = Peripherals::Timers::ChannelB<Peripherals::Timers::Timer2>;
        static constexpr uint8_t digital_pin_bit = PD3;
        static constexpr uint8_t id              = 3;
    };

    class D4 {
      public:
        using Port                               = Peripherals::Ports::PortD;
        static constexpr uint8_t digital_pin_bit = PD4;
        static constexpr uint8_t id              = 4;
    };

    class D5 {
      public:
        using Port                               = Peripherals::Ports::PortD;
        using TimerChannel                       = Peripherals::Timers::ChannelB<Peripherals::Timers::Timer0>;
        static constexpr uint8_t digital_pin_bit = PD5;
        static constexpr uint8_t id              = 5;
    };

    class D6 {
      public:
        using Port                               = Peripherals::Ports::PortD;
        using TimerChannel                       = Peripherals::Timers::ChannelA<Peripherals::Timers::Timer0>;
        using Tone                               = void;
        using AnalogCompPositive                 = Peripherals::Analog::AnalogComparator;
        static constexpr uint8_t digital_pin_bit = PD6;
        static constexpr uint8_t id              = 6;
    };

    class D7 {
      public:
        using Port                               = Peripherals::Ports::PortD;
        using AnalogCompNegative                 = Peripherals::Analog::AnalogComparator;
        static constexpr uint8_t digital_pin_bit = PD7;
        static constexpr uint8_t id              = 7;
    };

    class D8 {
      public:
        using Port                               = Peripherals::Ports::PortB;
        static constexpr uint8_t digital_pin_bit = PB0;
        static constexpr uint8_t id              = 8;
    };

    class D9 {
      public:
        using Port                               = Peripherals::Ports::PortB;
        using TimerChannel                       = Peripherals::Timers::ChannelA<Peripherals::Timers::Timer1>;
        using Tone                               = void;
        static constexpr uint8_t digital_pin_bit = PB1;
        static constexpr uint8_t id              = 9;
    };

    class D10 {
      public:
        using Port                               = Peripherals::Ports::PortB;
        using TimerChannel                       = Peripherals::Timers::ChannelB<Peripherals::Timers::Timer1>;
        static constexpr uint8_t digital_pin_bit = PB2;
        static constexpr uint8_t id              = 10;
    };

    class D11 {
      public:
        using Port                               = Peripherals::Ports::PortB;
        using TimerChannel                       = Peripherals::Timers::ChannelA<Peripherals::Timers::Timer2>;
        using Tone                               = void;
        static constexpr uint8_t digital_pin_bit = PB3;
        static constexpr uint8_t id              = 11;
    };

    class D12 {
      public:
        using Port                               = Peripherals::Ports::PortB;
        static constexpr uint8_t digital_pin_bit = PB4;
        static constexpr uint8_t id              = 12;
    };

    class D13 {
      public:
        using Port                               = Peripherals::Ports::PortB;
        static constexpr uint8_t digital_pin_bit = PB5;
        static constexpr uint8_t id              = 13;
    };

    using LedBuiltin = D13;

    class A0 {
      public:
        using AnalogConv                                                         = void;
        using Port                                                               = Peripherals::Ports::PortC;
        static constexpr uint8_t                                 digital_pin_bit = PC0;
        static constexpr uint8_t                                 id              = 14;
        static constexpr Peripherals::Analog::MultiplexerChannel analog_channel =
            Peripherals::Analog::MultiplexerChannel::ADC0;
    };

    class A1 {
      public:
        using AnalogConv                                                         = void;
        using Port                                                               = Peripherals::Ports::PortC;
        static constexpr uint8_t                                 digital_pin_bit = PC1;
        static constexpr uint8_t                                 id              = 15;
        static constexpr Peripherals::Analog::MultiplexerChannel analog_channel =
            Peripherals::Analog::MultiplexerChannel::ADC1;
    };

    class A2 {
      public:
        using AnalogConv                                                         = void;
        using Port                                                               = Peripherals::Ports::PortC;
        static constexpr uint8_t                                 digital_pin_bit = PC2;
        static constexpr uint8_t                                 id              = 16;
        static constexpr Peripherals::Analog::MultiplexerChannel analog_channel =
            Peripherals::Analog::MultiplexerChannel::ADC2;
    };

    class A3 {
      public:
        using AnalogConv                                                         = void;
        using Port                                                               = Peripherals::Ports::PortC;
        static constexpr uint8_t                                 digital_pin_bit = PC3;
        static constexpr uint8_t                                 id              = 17;
        static constexpr Peripherals::Analog::MultiplexerChannel analog_channel =
            Peripherals::Analog::MultiplexerChannel::ADC3;
    };

    class A4 {
      public:
        using AnalogConv                                                         = void;
        using Port                                                               = Peripherals::Ports::PortC;
        static constexpr uint8_t                                 digital_pin_bit = PC4;
        static constexpr uint8_t                                 id              = 18;
        static constexpr Peripherals::Analog::MultiplexerChannel analog_channel =
            Peripherals::Analog::MultiplexerChannel::ADC4;
    };

    class A5 {
      public:
        using AnalogConv                                                         = void;
        using Port                                                               = Peripherals::Ports::PortC;
        static constexpr uint8_t                                 digital_pin_bit = PC5;
        static constexpr uint8_t                                 id              = 19;
        static constexpr Peripherals::Analog::MultiplexerChannel analog_channel =
            Peripherals::Analog::MultiplexerChannel::ADC5;
    };

    class A6 {
      public:
        using AnalogConv                                            = void;
        static constexpr uint8_t                                 id = 20;
        static constexpr Peripherals::Analog::MultiplexerChannel analog_channel =
            Peripherals::Analog::MultiplexerChannel::ADC6;
    };

    class A7 {
      public:
        using AnalogConv                                            = void;
        static constexpr uint8_t                                 id = 21;
        static constexpr Peripherals::Analog::MultiplexerChannel analog_channel =
            Peripherals::Analog::MultiplexerChannel::ADC7;
    };
} // namespace Pin

#endif // STRONG_PINS_328P_H
