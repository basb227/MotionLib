/**
 * Copyright (c) 2020 Bas Brussen
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.                                                                         
 * 
 * @file Utils.hpp
 *
 * @brief The utils header holds supporting functions whch are used for calculating the motions.
 *
 * @author Bas Brussen
 * Contact: b.brussen@outlook.com
 *
 */

#ifndef Utils_hpp
#define Utils_hpp

#include <cmath>
#include <array>

#include "Definitions.hpp"

#define pi 3.14159265359
#define pi_d 1 / pi


template <typename T, size_t N>
struct Utils{
    virtual ~Utils() {}
    
    T dot(std::array<T, N> a, std::array<T, N> b) {
        T result = 0;

        for (size_t i = 0; i < N; i++)
            result += (a[i] * b[i]);

        return result;
    }

    T norm(std::array<T, N> a) { 
        T sum = 0.0; 

        for (size_t i = 0; i < N; i++)
            sum += (a[i] * a[i]); 

        return powl(sum, 0.5); 
    } 

    T norm(Point<T, N> p) { 
        T sum = 0; 

        for (size_t i = 0; i < N; i++)
            sum += (p.dim[i] * p.dim[i]);  

        return powl(sum, 0.5); 
    } 

    T angle_ratio(Point<T, N>& a, Point<T, N>& b, Point<T, N>& c) {
        // Calculate the delta's between b-a and b-c
        std::array<T, N> ba;
        std::array<T, N> bc;

        for (size_t i = 0; i < N; i++)
            ba[i] = a.dim[i] - b.dim[i]; 
        
        for (size_t i = 0; i < N; i++)
            bc[i] = c.dim[i] - b.dim[i]; 

        T ratio = std::fabs(dot(ba, bc) / (norm(ba) * norm(bc)));
        ratio = powf(ratio, CORNER_VELOCITY_RATIO) / pi_d;

        // Smallest corner ratio allowed.
        if (ratio < CORNER_MAX_RATIO){
            ratio = CORNER_MAX_RATIO;
        }
        else if (std::isnan(ratio) or std::isinf(ratio)) {
            ratio = CORNER_MAX_RATIO;
        }
        
        return ratio;
    }

    inline std::array<T, N> delta_array (std::array<T, N>& a, std::array<T, N>& b) {
        std::array<T, N> result;

        for (size_t i = 0; i < N; i++)
            result[i] = a[i] - b[i];

        return result;
    }

    inline std::array<T, N> multiply_array (std::array<T, N>& a, T& b) {
        std::array<T, N> result;

        for (size_t i = 0; i < N; i++)
            result[i] = a[i] * b;

        return result;
    }

    inline std::array<T, N> unit_vector(std::array<T, N> vec){
        std::array<T, N> result;

        T normal = norm(vec);

        for (size_t i = 0; i < N; i++)
            result[i] = vec[i] / normal;

        return result;
    }

    std::array<T, N> unit_vector(Point<T, N>& p){
        std::array<T, N> result;

        T normal = norm(p);

        for (size_t i = 0; i < N; i++)
            result[i] = p.dim[i] / normal;

        return result;
    }

    T integrate(T v_begin, T v, T v_prev, T dt){
        return ((v_begin + (v - v_prev) * 0.5) * dt);
    }

    T discrete(T t, int hz){
        hz = static_cast<T> (hz);
        return static_cast<T>(trunc(t * hz) * (1.0 / hz));
    }

    T sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
};

#endif