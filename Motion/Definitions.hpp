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
#include "ArrayMath.hpp"


/**
 * Defines a carthesian point.
 */
template <typename T, size_t N>
struct Point {
    std::array<T, N> setpoint {};
    std::array<T, N> p_prev {};
    T velocity {}, acceleration {};
    
    Point() : 
        velocity(0), 
        acceleration(0) {}

    Point(T velocity, T acceleration) : 
        velocity(velocity), 
        acceleration(acceleration) {}

    Point(std::array<T, N> setpoint) : 
        setpoint(setpoint), 
        velocity(0), 
        acceleration(0) {}
    
    Point(std::array<T, N> setpoint, T velocity, T acceleration) : 
        setpoint(setpoint), 
        velocity(velocity), 
        acceleration(acceleration) {}

    std::array<T, N> operator- (Point<T, N>& p) {
        return ml::min(this->setpoint, p.setpoint);
    }

    Point<T, N>& operator= (Point<T, N> p) {
        setpoint = p.setpoint;
        velocity = p.velocity;
        acceleration = p.acceleration;

        return *this;
    }
};

/**
 * Complete motion type for a Nth dimensional carthesian motion.
 * The polynomial is inherited and used to calculate velocities on the go.
 */
template <typename T, size_t N>
struct MotionObject : public Polynomial<T> {
    std::array<T, N> unit_vector {};
    std::array<T, N> prev_setpoint {};
    bool is_coast {false};

    T v_target {0.0};
    T dt {0.0};

    int n {0};

    MotionObject() {}
    virtual ~MotionObject() {}

    void reset(){
        is_coast = false;
        v_target = 0.0;
        dt = 0.0;
        n = 0;
        this->p_0 = 0;
    }

    T get_acceleration(int _n, int i) {
        return (this->polynomial_a(dt * _n) * unit_vector[i]);
    }

    T get_velocity(int _n, int i) {
        if (is_coast)
            return (v_target * unit_vector[i]);
        return (this->polynomial_v(dt * _n) * unit_vector[i]);
    }

    T get_position(int _n, int i) {
        if (is_coast) 
            return ((this->p_0 + this->v_target * (dt * _n)) * unit_vector[i]) + prev_setpoint[i];         
        return (this->polynomial_p(dt * _n) * unit_vector[i]) + prev_setpoint[i];
    }

    MotionObject<T, N>& operator= (MotionObject<T, N> m) {
        is_coast = m.is_coast;
        unit_vector = m.unit_vector;
        v_target = m.v_target;
        dt = m.dt;
        n = m.n;
        prev_setpoint = m.prev_setpoint;

        this->c_3 = m.c_3;
        this->c_4 = m.c_4;
        this->c_5 = m.c_5;
        this->c_6 = m.c_6;
        this->v_0 = m.v_0;
        this->p_0 = m.p_0;

        return *this;
    }
};

#endif