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
struct has_timer<T, void_t<typename T::Timer>> : true_type {};

///////////////////
/// Analog Requirement
///////////////////

template<class T, class=void>
struct is_analog : false_type {};

template<class T>
struct is_analog<T, void_t<typename T::Analog>> : true_type {};

#endif // STRONG_IO_TRAITS_H