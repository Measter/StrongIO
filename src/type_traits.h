#ifndef STRONG_IO_TRAITS_H
#define STRONG_IO_TRAITS_H

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