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
        class Timer0 {
            public:
                volatile uint8_t* control_a = &TCCR0A;
                volatile uint8_t* control_b = &TCCR0B;
                volatile uint8_t* interrupt_mask = &TIMSK0;
                volatile uint8_t* interupt_flag = &TIFR0;
                volatile uint8_t* counter_value = &TCNT0;
        };

        class Timer1 {
            public:
                volatile uint8_t* control_a = &TCCR1A;
                volatile uint8_t* control_b = &TCCR1B;
                volatile uint8_t* interrupt_mask = &TIMSK1;
                volatile uint8_t* interupt_flag = &TIFR1;
                volatile uint8_t* counter_value = &TCNT1L;
                volatile uint8_t* counter_value_high = &TCNT1H;
                volatile uint8_t* input_capture = &ICR1L;
                volatile uint8_t* input_capture_high = &ICR1H;
        };

        class Timer2 {
            public:
                volatile uint8_t* control_a = &TCCR2A;
                volatile uint8_t* control_b = &TCCR2B;
                volatile uint8_t* interrupt_mask = &TIMSK2;
                volatile uint8_t* interupt_flag = &TIFR2;
                volatile uint8_t* counter_value = &TCNT2;
        };

        class Timer0ChannelA {
            public:
                using Timer = Timer0;
                volatile uint8_t* output_compare = &OCR0A;
                static constexpr uint8_t mode_bit0 = 1 << COM0A0;
                static constexpr uint8_t mode_bit1 = 1 << COM0A1;
        };

        class Timer0ChannelB {
            public:
                using Timer = Timer0;
                volatile uint8_t* output_compare = &OCR0B;
                static constexpr uint8_t mode_bit0 = 1 << COM0B0;
                static constexpr uint8_t mode_bit1 = 1 << COM0B1;
        };

        class Timer1ChannelA {
            public:
                using Timer = Timer1;
                volatile uint8_t* output_compare = &OCR1AL;
                volatile uint8_t* output_compare_high = &OCR1AH;
                static constexpr uint8_t mode_bit0 = 1 << COM1A0;
                static constexpr uint8_t mode_bit1 = 1 << COM1A1;
        };

        class Timer1ChannelB {
            public:
                using Timer = Timer1;
                volatile uint8_t* output_compare = &OCR1BL;
                volatile uint8_t* output_compare_high = &OCR1BH;
                static constexpr uint8_t mode_bit0 = 1 << COM1B0;
                static constexpr uint8_t mode_bit1 = 1 << COM1B1;
        };

        class Timer2ChannelA {
            public:
                using Timer = Timer2;
                volatile uint8_t* output_compare = &OCR2A;
                static constexpr uint8_t mode_bit0 = 1 << COM2A0;
                static constexpr uint8_t mode_bit1 = 1 << COM2A1;
                static constexpr uint8_t output_compare_enable_bit = 1 << OCIE2A;
        };

        class Timer2ChannelB {
            public:
                using Timer = Timer2;
                volatile uint8_t* output_compare = &OCR2B;
                static constexpr uint8_t mode_bit0 = 1 << COM2B0;
                static constexpr uint8_t mode_bit1 = 1 << COM2B1;
                static constexpr uint8_t output_compare_enable_bit = 1 << OCIE2B;
        };
    }

    class AnalogComparitor {
        public:
            volatile uint8_t* adc_control_register = &ADCSRB;
            volatile uint8_t* comp_control_register = &ACSR;
            volatile uint8_t* digital_disable_register = &DIDR1;
    };

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
            using Timer = Timers::Timer2ChannelB;
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
            using Timer = Timers::Timer0ChannelB;
            static constexpr uint8_t digital_pin_bit = 1<<PD5;
            static constexpr uint8_t id = 5;
    };

    class D6 {
        public:
            using Port = Ports::PortD;
            using Timer = Timers::Timer0ChannelA;
            using AnalogCompPositive = AnalogComparitor;
            static constexpr uint8_t digital_pin_bit = 1<<PD6;
            static constexpr uint8_t id = 6;
    };

    class D7 {
        public:
            using Port = Ports::PortD;
            using AnalogCompNegative = AnalogComparitor;
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
            using Timer = Timers::Timer1ChannelA;
            static constexpr uint8_t digital_pin_bit = 1<<PB1;
            static constexpr uint8_t id = 9;
    };

    class D10 {
        public:
            using Port = Ports::PortB;
            using Timer = Timers::Timer1ChannelB;
            static constexpr uint8_t digital_pin_bit = 1<<PB2;
            static constexpr uint8_t id = 10;
    };

    class D11 {
        public:
            using Port = Ports::PortB;
            using Timer = Timers::Timer2ChannelA;
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