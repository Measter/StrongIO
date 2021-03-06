#ifndef STRONG_IO_TRAITS_H
#define STRONG_IO_TRAITS_H

#include <avr/io.h>

template <typename T>
constexpr T build_bitmask() {
    return 0;
}

template <typename T>
constexpr T build_bitmask(T bit) {
    return (1 << bit);
}

template <typename T, typename... Type>
constexpr T build_bitmask(T bit, Type... bits) {
    return (1 << bit) | build_bitmask(bits...);
}

template <typename Type, uint16_t Addr, Type Reset = 0>
struct IOReg {
    public:
        inline static constexpr volatile Type* ptr() {
            return reinterpret_cast<volatile Type*>(Addr);
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
            return *ptr() & (1 << bit);
        }

        inline static constexpr void set_bit(uint8_t bit) {
            *ptr() |= (1 << bit);
        }

        inline static constexpr void clear_bit(uint8_t bit) {
            *ptr() &= ~(1 << bit);
        }

        // Note that the mask is inverted inside the function, and should
        // have the bits to be replaced set.
        // E.g. To replace bits 2 and 4, the mask should be 0b00010100
        // Generally use the build_bitmask function to build the mask.
        inline static constexpr void replace_bits(Type mask, Type new_data) {
            Type masked_reg = *ptr() & ~mask;
            *ptr() = (masked_reg | new_data);
        }

        template <typename... Bits>
        inline static constexpr void set_bits(Bits... bits) {
            *ptr() |= build_bitmask(bits...);
        }

        template <typename... Bits>
        inline static constexpr void clear_bits(Bits... bits) {
            *ptr() &= ~build_bitmask(bits...);
        }
};

/// My thanks to /u/boredcircuits on Reddit/r/cpp_questions
// for basically giving me the answer to making this work.

// This upper section copied from the C++ STL
template <class T, T v>
struct integral_constant {
    static constexpr T value = v;
    typedef T value_type;
    typedef integral_constant<T,v> type;
    constexpr operator T() { return v; }
};

typedef integral_constant<bool, false> false_type;
typedef integral_constant<bool, true> true_type;

template<bool B, class T = void>
struct enable_if {};
 
template<class T>
struct enable_if<true, T> { typedef T type; };

template<class... Ts>
struct make_void { typedef void type; };

template<class... Ts>
using void_t = typename make_void<Ts...>::type;

///////////////////
/// Port Requirement
///////////////////

template<class T, class=void>
struct is_digital : false_type {};

template<class T>
struct is_digital<T, void_t<typename T::Port>> : true_type {};

///////////////////
/// Timer Requirement
///////////////////

template<class T, class=void>
struct has_timer : false_type {};

template<class T>
struct has_timer<T, void_t<typename T::TimerChannel>> : true_type {};

///////////////////
/// Tone Requirement
///////////////////

template<class T, class=void>
struct has_tone : false_type {};

template<class T>
struct has_tone<T, void_t<typename T::Tone>> : true_type {};

///////////////////
/// AnalogConv Requirement
///////////////////

template<class T, class=void>
struct is_analog_adc : false_type {};

template<class T>
struct is_analog_adc<T, void_t<typename T::AnalogConv>> : true_type {};

///////////////////
/// AnalogComp Requirement
///////////////////

template<class T, class=void>
struct is_analog_ac_positive : false_type {};

template<class T>
struct is_analog_ac_positive<T, void_t<typename T::AnalogCompPositive>> : true_type {};

template<class T, class=void>
struct is_analog_ac_negative : false_type {};

template<class T>
struct is_analog_ac_negative<T, void_t<typename T::AnalogCompNegative>> : true_type {};

#endif // STRONG_IO_TRAITS_H