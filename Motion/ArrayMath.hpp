

#ifndef ArrayMath_hpp
#define ArrayMath_hpp

#include <type_traits>
#include <array>
#include <vector>
#include <algorithm>
#include <numeric>

#define pi_d 1/3.14159265359

// This header will perform all array arithmatics. It is designed with SFINEA in mind but it is not completely exception free.
// Dynamic behaviour is not tested with these templated functions.

namespace ml {
    // Typetrait to test if type is either STL vector or STL array.
    namespace is_stl_container_impl{
        template <typename T>                   struct is_stl_container:std::false_type{};
        template <typename T, std::size_t N>    struct is_stl_container<std::array    <T,N>>    :std::true_type{};
        template <typename... Args>             struct is_stl_container<std::vector   <Args...>>:std::true_type{};
    }

    //Type trait to utilize the implementation type traits as well as decay the type
    template <typename T> struct is_stl_container {
        static constexpr bool const value = is_stl_container_impl::is_stl_container<std::decay_t<T>>::value;
    };

    /** Base function that performs arithmatics with scalars. 
     * 
     * Template arguments:
     * @param OP    Type of the arithmatic function pointer (e.g. std::plus, std::minus etc.)
     * @param A     Type of the results which should be an enumeration.
     * @param T     Type of the scalar.
     */
    template<template < class > class OP, typename A, typename T>
    std::enable_if_t<std::is_arithmetic<T>::value, void> math (A& a, T t) {
        std::for_each(a.begin(), a.end(), [&](auto& i){
            i = OP<T>()(i, t);
        });
    }

    /** Base function that performs arithmatics with enumerations.
     * 
     * Template arguments:
     * @param OP    Type of the arithmatic function pointer (e.g. std::plus, std::minus etc.)
     * @param A     Type of the results which should be an enumeration.
     * @param T     Type of the enumeration.
     * 
     * Function arguments:
     * @param a     Result of the arithmatic operation.
     * @param t     Enumeration for the operation.
     */
    template<template < class > class OP, typename A, typename T>
    std::enable_if_t<is_stl_container<T>::value, void> math (A& a, T&& t) {
        // Define the fundamental type of the enumration (int, double, float etc.).
        typedef typename std::remove_reference<T &&>::type::value_type value_type;

        for (int i  = 0; i < a.size(); i++) {
            a[i] = OP<value_type>()(a[i], t[i]);
        }
    }

    /** Add function that performs addition arithmatics.
     * 
     * Template arguments:
     * @param A     Type of the results which should be an enumeration.
     * @param T     Type of the enumeration.
     * 
     * Function arguments:
     * @param a     Result of the arithmatic operation.
     * @param t     Enumeration or scalar for the operation.
     */
    template<typename A, typename ... Args>
    A add (A& a, Args&& ... args) {
        const auto arguments = {args...};
        A result = a;

        std::for_each(arguments.begin(), arguments.end(), [&](auto& i){
            // Perform operation for each argument.
            math<std::plus>(result, i);
        });

        return result;
    }

    /** Min function that performs minus arithmatics.
     * 
     * Template arguments:
     * @param A     Type of the results which should be an enumeration.
     * @param T     Type of the enumeration.
     * 
     * Function arguments:
     * @param a     Result of the arithmatic operation.
     * @param t     Enumeration or scalar for the operation.
     */
    template<typename A, typename ... Args>
    A min (A& a, Args&& ... args) {
        const auto arguments = {args...};
        A result = a;

        std::for_each(arguments.begin(), arguments.end(), [&](auto& i){
            // Perform operation for each argument.
            math<std::minus>(result, i);
        });

        return result;
    }

    /** Mul function that performs multiplication arithmatics.
     * 
     * Template arguments:
     * @param A     Type of the results which should be an enumeration.
     * @param T     Type of the enumeration.
     * 
     * Function arguments:
     * @param a     Result of the arithmatic operation.
     * @param t     Enumeration or scalar for the operation.
     */
    template<typename A, typename ... Args>
    A mul (A& a, Args&& ... args) {
        const auto arguments = {args...};
        A result = a;

        std::for_each(arguments.begin(), arguments.end(), [&](auto& i){
            // Perform operation for each argument.
            math<std::multiplies>(result, i);
        });

        return result;
    }

    /** Div function that performs division arithmatics.
     * 
     * Template arguments:
     * @param A     Type of the results which should be an enumeration.
     * @param T     Type of the enumeration.
     * 
     * Function arguments:
     * @param a     Result of the arithmatic operation.
     * @param t     Enumeration or scalar for the operation.
     */
    template<typename A, typename ... Args>
    A div (A& a, Args&& ... args) {
        const auto arguments = {args...};
        A result = a;

        std::for_each(arguments.begin(), arguments.end(), [&](auto& i){
            // Perform operation for each argument.
            math<std::divides>(result, i);
        });

        return result;
    }

    /** Accumulate function that performs accumulation arithmatic on enumeration.
     * 
     * Template arguments:
     * @param T     Type of the enumeration.
     * 
     * Function arguments:
     * @param t     Enumeration for the operation.
     */
    template<typename T>
    typename std::remove_reference<T &&>::type::value_type 
    accum (T&& t) {
        return std::accumulate(t.begin(), t.end(), 0);
    }

    /* Utiliary functions regarding array arithmatics */
    template<typename T, size_t N>
    T dot(std::array<T, N>& a, std::array<T, N>& b) {
        return ml::accum(ml::mul(a, b));
    }

    template<typename T, size_t N>
    T norm(std::array<T, N>& a) { 
        return powl(ml::accum(ml::mul(a, a)), 0.5); 
    } 

    template<typename T, size_t N>
    T angle_ratio(std::array<T, N>& a, std::array<T, N>& b, std::array<T, N>& c) {
        // Calculate the delta's between b-a and b-c
        std::array<T, N> ab {ml::min(a, b)};
        std::array<T, N> cb {ml::min(c, b)};

        T ratio = std::fabs(dot(ab, cb) / (norm(ab) * norm(cb)));
        ratio = powf(ratio, 5.0) / pi_d; // CORNER_VELOCITY_RATIO

        // Smallest corner ratio allowed.
        if (ratio < 0.01){ // CORNER_MAX_RATIO
            ratio = 0.01;  // CORNER_MAX_RATIO
        }
        else if (std::isnan(ratio) or std::isinf(ratio)) {
            ratio = 0.01;  // CORNER_MAX_RATIO
        }
        
        return ratio;
    }

    template<typename T, size_t N>
    inline std::array<T, N> delta_array (std::array<T, N>& a, std::array<T, N>& b) {
        return ml::min(a, b);
    }

    template<typename T, size_t N>
    inline std::array<T, N> multiply_array (std::array<T, N>& a, T& b) {
        return ml::mul(a, b);
    }

    template<typename T, size_t N>
    inline std::array<T, N> unit_vector(std::array<T, N>& vec){
        return ml::div(vec, norm(vec));
    }

    template<typename T>
    T integrate(T v_begin, T v, T v_prev, T dt){
        return ((v_begin + (v - v_prev) * 0.5) * dt);
    }

    template<typename T>
    inline T discrete(T t){
        return static_cast<T>(trunc(fabs(t)));
    }

    template<typename T>
    T sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
}

#endif