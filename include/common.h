#ifndef STRONG_IO_TRAITS_H
#define STRONG_IO_TRAITS_H

#include <avr/io.h>

template <typename T>
constexpr T bitmask(T bit) {
    return (1 << bit);
}

template <typename T, typename... Type>
constexpr T bitmask(T bit, Type... bits) {
    return static_cast<T>(1 << bit) | bitmask(bits...);
}

template <typename Type, uint16_t Addr, Type Reset = 0>
struct IOReg {
  public:
    inline static constexpr volatile Type *ptr() {
        return reinterpret_cast<volatile Type *>(Addr);
    }

    inline static constexpr void reset() {
        *ptr() = Reset;
    }

    inline static constexpr void set_value(Type val) {
        *ptr() = val;
    }

    inline static constexpr Type get_value() {
        return *ptr();
    }

    inline static constexpr Type get_bit(uint8_t bit) {
        return *ptr() & static_cast<Type>((1 << bit));
    }

    inline static constexpr void set_bit(uint8_t bit) {
        *ptr() = *ptr() | static_cast<Type>(1 << bit);
    }

    inline static constexpr void clear_bit(uint8_t bit) {
        *ptr() = *ptr() & static_cast<Type>(~(1 << bit));
    }

    // Note that the mask is inverted inside the function, and should
    // have the bits to be replaced set.
    // E.g. To replace bits 2 and 4, the mask should be 0b00010100
    // Generally use the bitmask function to build the mask.
    inline static constexpr void replace_bits(Type mask, Type new_data) {
        Type masked_reg = *ptr() & static_cast<Type>(~mask);
        *ptr()          = (masked_reg | new_data);
    }

    template <typename... Bits>
    inline static constexpr void set_bits(Bits... bits) {
        *ptr() = *ptr() | bitmask(bits...);
    }

    template <typename... Bits>
    inline static constexpr void clear_bits(Bits... bits) {
        *ptr() = *ptr() & ~bitmask(bits...);
    }
};

template <typename T>
concept DigitalPin = requires {
    typename T::Port;
    T::digital_pin_bit;
};

template <typename T>
concept TimerPin = DigitalPin<T> && requires {
    typename T::TimerChannel;
};

template <typename T>
concept TonePin = TimerPin<T> && requires {
    typename T::Tone;
};

template <typename T>
concept AdcPin = requires {
    typename T::AnalogConv;
    T::analog_channel;
};

template <typename T>
concept AcPositive = requires {
    typename T::AnalogCompPositive;
};

template <typename T>
concept AcNegative = requires {
    typename T::AnalogCompNegative;
};

template <class T, T v>
struct integral_constant {
    static constexpr T              value = v;
    typedef T                       value_type;
    typedef integral_constant<T, v> type;

    constexpr operator T() {
        return v;
    }
};

typedef integral_constant<bool, false> false_type;
typedef integral_constant<bool, true>  true_type;

template <class T, class U>
struct is_same: false_type {};

template <class T>
struct is_same<T, T>: true_type {};

template <class T, class U>
concept IsSame = is_same<T, U>::value && is_same<U, T>::value;

#endif // STRONG_IO_TRAITS_H