

#ifndef ArrayMath_hpp
#define ArrayMath_hpp

#include <type_traits>
#include <array>
#include <vector>
#include <algorithm>
#include <numeric>

namespace ml {
    namespace is_stl_container_impl{
        template <typename T>       struct is_stl_container:std::false_type{};
        template <typename T, std::size_t N> struct is_stl_container<std::array    <T,N>>    :std::true_type{};
        template <typename... Args> struct is_stl_container<std::vector            <Args...>>:std::true_type{};
    }

    //type trait to utilize the implementation type traits as well as decay the type
    template <typename T> struct is_stl_container {
        static constexpr bool const value = is_stl_container_impl::is_stl_container<std::decay_t<T>>::value;
    };


    template<template < class > class OP, typename A, typename T>
    std::enable_if_t<std::is_arithmetic<T>::value, void> math (A& a, T t) {
        std::for_each(a.begin(), a.end(), [&](auto& i){
            i = OP<T>()(i, t);
        });
    }

    template<template < class > class OP, typename A, typename T>
    std::enable_if_t<is_stl_container<T>::value, void> math (A& a, T&& t) {
        
        typedef typename std::remove_reference<T &&>::type::value_type value_type;

        for (int i  = 0; i < a.size(); i++) {
            a[i] = OP<value_type>()(a[i], t[i]);
        }
    }

    template<typename A, typename ... Args>
    A add (A& a, Args ... args) {
        auto arguments = {args...};
        A result = a;

        std::for_each(arguments.begin(), arguments.end(), [&](auto& i){
            math<std::plus>(result, i);
        });

        return result;
    }

    template<typename A, typename ... Args>
    A min (A& a, Args ... args) {
        auto arguments = {args...};
        A result = a;

        std::for_each(arguments.begin(), arguments.end(), [&](auto& i){
            math<std::minus>(result, i);
        });

        return result;
    }

    template<typename A, typename ... Args>
    A mul (A& a, Args ... args) {
        auto arguments = {args...};
        A result = a;

        std::for_each(arguments.begin(), arguments.end(), [&](auto& i){
            math<std::multiplies>(result, i);
        });

        return result;
    }

    template<typename A, typename ... Args>
    A div (A& a, Args ... args) {
        auto arguments = {args...};
        A result = a;

        std::for_each(arguments.begin(), arguments.end(), [&](auto& i){
            math<std::divides>(result, i);
        });

        return result;
    }

    template<typename T>
    typename std::remove_reference<T &&>::type::value_type 
    accum (T&& t) {
        return std::accumulate(t.begin(), t.end(), 0);
    }
}

#endif