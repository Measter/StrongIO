#ifndef STRONG_PINS_H
#define STRONG_PINS_H

namespace Pin {
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
        class Timer0A {
            public:
                volatile uint8_t* control_register = &TCCR0A;
                volatile uint8_t* compare_register = &OCR0A;
                static constexpr uint8_t mode_bit0 = 1 << 6;
                static constexpr uint8_t mode_bit1 = 1 << 7;
        };

        class Timer0B {
            public:
                volatile uint8_t* control_register = &TCCR0A;
                volatile uint8_t* compare_register = &OCR0B;
                static constexpr uint8_t mode_bit0 = 1 << 4;
                static constexpr uint8_t mode_bit1 = 1 << 5;
        };

        class Timer1A {
            public:
                volatile uint8_t* control_register = &TCCR1A;
                volatile uint8_t* compare_register = &OCR1AL;
                static constexpr uint8_t mode_bit0 = 1 << 6;
                static constexpr uint8_t mode_bit1 = 1 << 7;
        };

        class Timer1B {
            public:
                volatile uint8_t* control_register = &TCCR1A;
                volatile uint8_t* compare_register = &OCR1BL;
                static constexpr uint8_t mode_bit0 = 1 << 4;
                static constexpr uint8_t mode_bit1 = 1 << 5;
        };

        class Timer2A {
            public:
                volatile uint8_t* control_register = &TCCR2A;
                volatile uint8_t* compare_register = &OCR2A;
                static constexpr uint8_t mode_bit0 = 1 << 6;
                static constexpr uint8_t mode_bit1 = 1 << 7;
        };

        class Timer2B {
            public:
                volatile uint8_t* control_register = &TCCR2A;
                volatile uint8_t* compare_register = &OCR2B;
                static constexpr uint8_t mode_bit0 = 1 << 4;
                static constexpr uint8_t mode_bit1 = 1 << 5;
        };
    }

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
            using Timer = Timers::Timer2B;
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
            using Timer = Timers::Timer0B;
            static constexpr uint8_t digital_pin_bit = 1<<PD5;
            static constexpr uint8_t id = 5;
    };

    class D6 {
        public:
            using Port = Ports::PortD;
            using Timer = Timers::Timer0A;
            static constexpr uint8_t digital_pin_bit = 1<<PD6;
            static constexpr uint8_t id = 6;
    };

    class D7 {
        public:
            using Port = Ports::PortD;
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
            using Timer = Timers::Timer1A;
            static constexpr uint8_t digital_pin_bit = 1<<PB1;
            static constexpr uint8_t id = 9;
    };

    class D10 {
        public:
            using Port = Ports::PortB;
            using Timer = Timers::Timer1B;
            static constexpr uint8_t digital_pin_bit = 1<<PB2;
            static constexpr uint8_t id = 10;
    };

    class D11 {
        public:
            using Port = Ports::PortB;
            using Timer = Timers::Timer2A;
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

#endif //STRONG_PINS_H