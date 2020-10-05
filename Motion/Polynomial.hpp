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
 * @file Polynomial.hpp
 *
 * @brief The Polynomial header holds the actual polynomial function which is used to calculate the velocity profiles.
 *
 * @author Bas Brussen
 * Contact: b.brussen@outlook.com
 *
 */

#include <math.h>

template <typename T>
struct Polynomial {
    static constexpr T pol_p_c = 1.0 / 420.0;

    T c_3;
    T c_4;
    T c_5;
    T c_6;
    T v_0;
    
    Polynomial() : 
    c_3(1.), c_4(1.), c_5(1.), c_6(1.), v_0(0.) {}

    ~Polynomial(){}

    inline void calc_constants(T v_f, T t){
        T v_v = v_f * 0.5;

        c_3 = 2. * (32. * v_v - 11. * v_f) / (t * t * t);
        c_4 = -3. * (64. * v_v - 27. * v_f) / (t * t * t * t);
        c_5 = 3. * (64. * v_v - 30. * v_f) / (t * t * t * t * t);
        c_6 = -32. * (2. * v_v - v_f) / (t * t * t * t * t * t);
    }

    inline void calc_constants(T v_v, T v_f, T t){
        c_3 = 2. * (32. * v_v - 11. * v_f) / (t * t * t);
        c_4 = -3. * (64. * v_v - 27. * v_f) / (t * t * t * t);
        c_5 = 3. * (64. * v_v - 30. * v_f) / (t * t * t * t * t);
        c_6 = -32. * (2. * v_v - v_f) / (t * t * t * t * t * t);
    }

    inline void calc_constants_v(T v_s, T v_f, T t){
        T v_h = v_f > v_s ? v_f : v_s;
        T v_l = v_f < v_s ? v_f : v_s;

        T v_v = (v_h - v_l) * 0.5;

        v_0 = v_s;

        c_3 = 2. * (32. * (v_v - v_s) - 11. * (v_f - v_s)) / (t * t * t);
        c_4 = -3. * (64. * (v_v - v_s) - 27. * (v_f - v_s)) / (t * t * t * t);
        c_5 = 3. * (64. * (v_v - v_s) - 30. * (v_f - v_s)) / (t * t * t * t * t);
        c_6 = -32. * (2. * (v_v - v_s) - (v_f - v_s)) / (t * t * t * t * t * t);
    }

    inline void calc_constants_v(T v_s, T v_v, T v_f, T t){
        v_0 = v_s;

        c_3 = 2. * (32. * (v_v - v_s) - 11. * (v_f - v_s)) / (t * t * t);
        c_4 = -3. * (64. * (v_v - v_s) - 27. * (v_f - v_s)) / (t * t * t * t);
        c_5 = 3. * (64. * (v_v - v_s) - 30. * (v_f - v_s)) / (t * t * t * t * t);
        c_6 = -32. * (2. * (v_v - v_s) - (v_f - v_s)) / (t * t * t * t * t * t);
    }

    inline T polynomial_p(T t){
        return  pol_p_c * t * (105 * c_3 * (t * t * t) + 
                2 * (42 * c_4 * (t * t * t * t) + 
                5 * (6 * (c_6 * (t * t * t* t * t * t) + 7 * v_0) + 
                7 * c_5 * (t * t * t* t * t))));
    } 

    inline T polynomial_v(T t){
        return (t * t * t) * (t * (t * (c_6 * t + c_5) + c_4) + c_3) + v_0;
    } 

    inline T polynomial_a(T t){
        return (t * t) * (t * (6. * c_6 * (t * t) + 5. * c_5 * t + 4 * c_4) + 3. * c_3);
    };   
};