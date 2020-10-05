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
 * @file MotionPlanner.hpp
 *
 * @brief The motion planner header holds the class which calcualtes the polynomials of the motions.
 *
 * @author Bas Brussen
 * Contact: b.brussen@outlook.com
 *
 */

#include <math.h>

#include "MotionHandler.hpp"

template <typename T, size_t N>
class MotionPlanner : public MotionHandler<T, N>, public Utils<T, N> {
public:
    MotionPlanner(const int hz) : hz(hz), dt(1./hz) {
        unit_prev.fill(0);

        Point<T, N> p(STANDARD_VELOCITY, STANDARD_ACCELERATION);
        append_buffer(p);
        append_buffer(p);
        append_buffer(p);
    }

    virtual ~MotionPlanner(){};

    void do_command(const Point<T, N>& p){
        // First append required to fill buffer.
        append_buffer(p);
        plan_motion();
    }

private:
    MotionObject<T, N> current_motion;
    std::array<Point<T, N>, 3> mp_buffer;    // Motion buffer containing three appended motions (G0, G1).
    std::array<T, N> unit_prev;

    const int hz;
    const T dt;

    T v_enter {0.0};
    T delta_p {0.0};

    void append_buffer(const Point<T, N>& p){
        mp_buffer[0] = mp_buffer[1];
        mp_buffer[1] = mp_buffer[2]; 
        mp_buffer[2] = std::move(p);
    }

    void plan_motion(){
        // Calculate delta's of axis.
        auto delta_unit {this->unit_vector(mp_buffer[1] - mp_buffer[0])};
        auto carthesian_delta {this->norm(mp_buffer[1] - mp_buffer[0])};

        // Check for second motion entry.
        if (carthesian_delta == 0.0) {
            append_buffer(mp_buffer[2]);
            delta_unit = this->unit_vector(mp_buffer[1] - mp_buffer[0]);
            carthesian_delta = this->norm(mp_buffer[1] - mp_buffer[0]);
            // If delta is 0, nothing to calculate.
            if (carthesian_delta == 0.0)
                return;
        }

        T ratio {this->angle_ratio(mp_buffer[0], mp_buffer[1], mp_buffer[2])};
            
        T v_exit {mp_buffer[1].vel * ratio};           // Velocity at end of trajectory (or final velocity).
        T v_target {mp_buffer[1].vel};
        T a_target {mp_buffer[1].acc};
        
        T v_delta {v_exit - v_enter}; 
        T v_delta_target {v_target - v_enter};      // Delta velocity for acceleration phase.
        T v_delta_exit {v_exit - v_target};         // Delta velocity for deceleration phase.
        
        // Calculate the time and distance required to reach target velocities.
        T t_acc {this->calc_accel_time (v_delta_target, a_target)};
        T p_acc {this->calc_accel_position (v_enter, v_target, t_acc)};

        T t_dec {this->calc_accel_time (v_delta_exit, a_target)};
        T p_dec {abs(this->calc_accel_position (v_target, v_exit, t_dec))};

        // Determine if the first and second acceleration event in the motion is bigger than the total distance.
        // If its true, coasting motion is calculated, if false, transition motion id calculated.
        if ((carthesian_delta < 1) or ((p_acc + p_dec) > carthesian_delta))
            transition(carthesian_delta, v_enter, v_target, v_delta, a_target, delta_unit, v_exit);
        
        else 
            motion(v_enter, v_target, v_exit, carthesian_delta, p_acc, p_dec, t_acc, t_dec, delta_unit);

        v_enter = v_exit;
    }  

    /** Motion Generating functions
    * Calculate time required to change velocity constrained by acceleration.
    * The constants of the polynomial is updated after t is calculated.

    * @param v_delta Velocity change.
    * @param a_target Maximum acceleration.
    * @return Time required to change velocity and position that is reached.
    */
    T calc_accel_time(const T& v_delta, const T& a_target){
        current_motion.calc_constants(v_delta, 1.0);

        // Calculate the time in respect to the discrete timing.
        // This means that the time should be rounded so an integral number of samples can be calculated from is.
        return static_cast<T>(trunc(fabs(current_motion.polynomial_a(0.5) / a_target) * hz) * dt);
    }

    T calc_accel_position (const T& v_enter, const T& v_target, const T& t) {
        current_motion.calc_constants_v(v_enter, v_target, t);
        return current_motion.polynomial_p(t);
    }

    void validate_position (T& v_target, const T& position, const T& t) {
        v_target *= (position / current_motion.polynomial_p(t));
    }

    void transition (const T& p_delta, const T& v_enter, T v_target, const T& v_delta, const T& a_target, std::array<T, N>& delta_unit, T v_exit){
        T t {0};
        T p {0};
        T ratio {0};

        // Lambda to calculate the ratio between the intial motion constraint to 
        // acceleratin and the required motion to reach the position specified.
        auto get_ratio = [&] (auto v_t, auto v_e) {
            t = calc_accel_time((v_t - v_e), a_target);
            current_motion.calc_constants_v(v_e, v_t, t);
            p = current_motion.polynomial_p(t);
            return (((p_delta * 0.5) - delta_p) / p);
        };
        
        // First part of the transition
        ratio = get_ratio(v_target, v_enter);
        v_target *= ratio;
        current_motion.calc_constants_v(v_enter, v_target, t);

        validate_position(v_target, p_delta * 0.5, t);
        current_motion.calc_constants_v(v_enter, v_target, t);

        delta_p = current_motion.polynomial_p(t) - (p_delta * 0.5);

        update_motion (
            static_cast<int> (t * hz), 
            delta_unit, 
            v_target, 
            false
        );

        // Second part of the transition
        ratio = get_ratio(v_target, v_exit);
        t *= ratio;
        current_motion.calc_constants_v(v_target, v_exit, t);

        validate_position(v_exit, p_delta * 0.5, t);
        current_motion.calc_constants_v(v_target, v_exit, t);

        delta_p = current_motion.polynomial_p(t) - (p_delta * 0.5);

        update_motion (
            static_cast<int> (t * hz), 
            delta_unit, 
            v_target, 
            false
        );
    }

    void motion (const T& v_enter, const T& v_target, const T& v_exit, 
                 const T& p_delta_carthesian, const T& p_acc, const T& p_dec, 
                 const T& t_acc, const T& t_dec, const std::array<T, N>& delta_unit) {
        // Calculate the accelerating phase
        current_motion.calc_constants_v(v_enter, v_target, t_acc);
        //p_acc = current_motion.polynomial_p(t_acc);

        update_motion (
            static_cast<int> (t_acc * this->hz),
            delta_unit,
            v_target,
            false
        );

        // Calculate the coasting phase
        T t {static_cast<T>(trunc(fabs((p_delta_carthesian - p_dec - p_acc - delta_p) / v_target) * hz) * (dt))};
        T p_coast {t * v_target}; 
        delta_p = p_delta_carthesian - p_acc - p_dec - p_coast;

        t = static_cast<T>(trunc(fabs((p_coast - delta_p) / v_target) * hz) * (dt));

        update_motion (
            static_cast<int> (t * this->hz),
            delta_unit,
            v_target,
            true
        );

        // Calculate deceleration phase
        current_motion.calc_constants_v(v_target, v_exit, t_dec);

        update_motion (
            static_cast<int> (t_dec * this->hz),
            delta_unit,
            v_target,
            false
        );
    }

    void update_motion (int n, const std::array<T, N>& unit_vec, T velocity, bool is_coast) {
        current_motion.n = n;
        current_motion.dt = dt;
        current_motion.unit = unit_vec;
        current_motion.v_target = velocity;
        current_motion.is_coast = is_coast;
        
        this->append_motion(current_motion);

        current_motion.reset();
    }
};