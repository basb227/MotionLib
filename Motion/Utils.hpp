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

#include "Config.hpp"
#include "ArrayMath.hpp"

#define pi 3.14159265359
#define pi_d 1 / pi


template <typename T, size_t N>
struct Utils{
    virtual ~Utils() {}
    
    T dot(std::array<T, N> a, std::array<T, N> b) {
        return ml::accum(ml::mul(a, b));
    }

    T norm(std::array<T, N> a) { 
        return powl(ml::accum(ml::mul(a, a)), 0.5); 
    } 

    T angle_ratio(std::array<T, N>& a, std::array<T, N>& b, std::array<T, N>& c) {
        // Calculate the delta's between b-a and b-c
        std::array<T, N> ab {ml::min(a, b)};
        std::array<T, N> cb {ml::min(c, b)};

        T ratio = std::fabs(dot(ab, cb) / (norm(ab) * norm(cb)));
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
        return ml::min(a, b);
    }

    inline std::array<T, N> multiply_array (std::array<T, N>& a, T& b) {
        return ml::mul(a, b);
    }

    inline std::array<T, N> unit_vector(std::array<T, N> vec){
        return ml::div(vec, norm(vec));
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