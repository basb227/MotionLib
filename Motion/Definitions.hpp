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
 * @file Definitions.hpp
 *
 * @brief Definitions hold objects used to define motions and points.
 *
 * @author Bas Brussen
 * Contact: b.brussen@outlook.com
 *
 */

#ifndef Definitions_hpp
#define Definitions_hpp

#include <utility>

#include "Polynomial.hpp"
#include "Config.hpp"

/**
 * Defines a carthesian point.
 */
template <typename T, size_t N>
struct Point {
    std::array<T, N> dim;
    T vel, acc;
    
    Point() : vel(0), acc(0) {dim.fill(0.);}

    Point(T f, T a) : 
    vel(f), acc(a) {dim.fill(0.);}

    Point(std::array<T, N> dim) : 
    dim(dim), vel(STANDARD_VELOCITY), acc(STANDARD_ACCELERATION) {}
    
    Point(std::array<T, N> dim, T f, T a) : 
    dim(dim), vel(f), acc(a) {}

    virtual std::array<T, N> operator- (Point<T, N>& p) {
        std::array<T, N> result;

        for (size_t i = 0 ; i < N ; i++) 
            result[i] = this->dim[i] - p.dim[i];
        
        return result;
    }
};

/**
 * Complete motion type for a Nth dimensional carthesian motion.
 * The polynomial is inherited and used to calculate velocities on the go.
 */
template <typename T, size_t N>
struct MotionObject : public Polynomial<T> {
    std::array<T, N> unit;
    bool is_coast {false};

    T v_target {0.0};
    T dt {0.0};

    int n {0};

    MotionObject() {unit.fill(0);}
    virtual ~MotionObject() {}

    void reset(){
        is_coast = false;
        unit.fill(0.0);
        v_target = 0.0;
        dt = 0.0;
        n = 0;
    }

    T get_velocity(int _n, int i) {
        if (is_coast)
            return (v_target * unit[i]);
        return (this->polynomial_v(dt * _n) * unit[i]);
    }

    T get_position(int _n, int i) {
        return (this->polynomial_p(dt * _n) * unit[i]);
    }

    void operator = (MotionObject<T, N> m) {
        is_coast = m.is_coast;
        unit = m.unit;
        v_target = m.v_target;
        dt = m.dt;
        n = m.n;

        this->c_3 = m.c_3;
        this->c_4 = m.c_4;
        this->c_5 = m.c_5;
        this->c_6 = m.c_6;
        this->v_0 = m.v_0;
    }
};

#endif