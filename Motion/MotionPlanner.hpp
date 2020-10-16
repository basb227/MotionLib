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

#ifndef MotionPlanner_hpp
#define MotionPlanner_hpp

#include "SetpointBuffer.hpp"
#include "MotionHandler.hpp"

template <typename T, size_t N>
class MotionPlanner : public MotionHandler<T, N>, public SetpointBuffer<T, N> {
public:
    int hz;
    T dt;

    MotionPlanner(const int hz) : 
        hz(hz), 
        dt(1./hz) { }

    MotionPlanner(const int hz, std::array<T, N>& point) : 
        SetpointBuffer<T, N>(point),
        hz(hz), 
        dt(1./hz) { }

    virtual ~MotionPlanner(){};

    void plan(const Point<T, N>& p){
        // First append required to fill buffer.
        this->append_buffer(p);
        plan_motion();
    }

    void plan(const Point<T, N>& p, T& v_final){
        // First append required to fill buffer.
        this->append_buffer(p);
        plan_motion(v_final);
    }

private:
    MotionObject<T, N> current_motion;   

    T v_enter {0.0};
    T error {0.0};

    void plan_motion(){
        // Calculate delta's of axis.
        auto m = ml::min(this->mp_buffer[1].setpoint, 
                         this->mp_buffer[0].setpoint);
        auto delta_unit {ml::unit_vector(m)};
        auto carthesian_delta {ml::norm(m)};

        // Check for second motion entry.
        if (carthesian_delta < 1e-9)
            return;

        T ratio {ml::angle_ratio(this->mp_buffer[0].setpoint, 
                                 this->mp_buffer[1].setpoint, 
                                 this->mp_buffer[2].setpoint)};
        
        T v_exit {this->mp_buffer[1].velocity * ratio};                         // Velocity at end of trajectory (or final velocity).
        T v_target {this->mp_buffer[1].velocity};              // Velocity which the planner will try to reach.
        T a_target {this->mp_buffer[1].acceleration};              // Accelerataion which the planner will try to reach.
        
        T v_delta {v_exit - v_enter};               // Delta velocity of the enter and exit velocities.
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
            // Transition motion calculates two motions and "transitions" to a velocity/position.
            transition(carthesian_delta, v_enter, v_target, a_target, delta_unit, v_exit, t_acc);
        
        else 
            // Motion will calculate three motions from v_enter, to v_target, to v_exit.
            motion(v_enter, v_target, v_exit, carthesian_delta, p_acc, p_dec, t_acc, t_dec, delta_unit);

        v_enter = v_exit;
    }  

    void plan_motion(T& v_final){
        // Calculate delta's of axis.
        auto m = ml::min(this->mp_buffer[1].setpoint, 
                         this->mp_buffer[0].setpoint);
        auto delta_unit {ml::unit_vector(m)};
        auto carthesian_delta {ml::norm(m)};

        // Check for second motion entry.
        if (carthesian_delta  < 1e-9)
            return;

        T ratio {ml::angle_ratio(this->mp_buffer[0].setpoint, 
                                 this->mp_buffer[1].setpoint, 
                                 this->mp_buffer[2].setpoint)};
        
        T v_exit {v_final};                         // Velocity at end of trajectory (or final velocity).
        T v_target {this->mp_buffer[1].velocity};              // Velocity which the planner will try to reach.
        T a_target {this->mp_buffer[1].acceleration};              // Accelerataion which the planner will try to reach.
        
        T v_delta {v_exit - v_enter};               // Delta velocity of the enter and exit velocities.
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
            // Transition motion calculates two motions and "transitions" to a velocity/position.
            transition(carthesian_delta, v_enter, v_target, a_target, delta_unit, v_exit, t_acc);
        
        else 
            // Motion will calculate three motions from v_enter, to v_target, to v_exit.
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

    void transition (const T& p_delta, const T& v_enter, T v_target, const T& a_target, std::array<T, N>& delta_unit, T v_exit, T& t_acc){
        T t {0};
        T p {0};
        T ratio {0};
        
        auto validate_position = [&] (T& v_target, const T& position, const T& t) {
            v_target *= (position / current_motion.polynomial_p(t));
        };
        
        // First part of the transition
        current_motion.calc_constants_v(v_enter , v_target, t_acc);
        // Get position ratio
        ratio = fabs(((p_delta * 0.5) - error) / current_motion.polynomial_p(t_acc));

        // Correct time and velocity based on this ratio
        v_target *= ratio;
        t = t_acc * ratio;

        // Calculate new constants
        current_motion.calc_constants_v(v_enter, v_target, t);
        validate_position(v_target, p_delta * 0.5, t);
        current_motion.calc_constants_v(v_enter, v_target, t);

        // Calculate the error.
        error = current_motion.polynomial_p(t) - (p_delta * 0.5);

        // p_0 as reference for next part of the motion.
        T p_0 = current_motion.polynomial_p(t);

        update_motion (
            static_cast<int> (t * hz), 
            delta_unit, 
            v_target, 
            {},
            false
        );

        // Second part of the transition
        t = calc_accel_time((v_target - v_exit), a_target);
        current_motion.calc_constants_v(v_exit, v_target, t);
        ratio = fabs(((p_delta * 0.5) - error) / current_motion.polynomial_p(t));
        t *= ratio;

        current_motion.calc_constants_v(v_target, v_exit, t);

        validate_position(v_exit, p_delta * 0.5, t);
        current_motion.calc_constants_v(v_target, v_exit, t);

        error = current_motion.polynomial_p(t) - (p_delta * 0.5);

        update_motion (
            static_cast<int> (t * hz), 
            delta_unit, 
            v_target, 
            p_0,
            false
        );
    }

    void motion (const T& v_enter, const T& v_target, const T& v_exit, 
                 const T& p_delta_carthesian, const T& p_acc, const T& p_dec, 
                 const T& t_acc, const T& t_dec, std::array<T, N>& delta_unit) {
        // Calculate the accelerating phase
        current_motion.calc_constants_v(v_enter, v_target, t_acc);

        update_motion (
            static_cast<int> (t_acc * this->hz),
            delta_unit,
            v_target,
            {},
            false
        );

        // Calculate the coasting phase
        T t {static_cast<T>(trunc(fabs((p_delta_carthesian - p_dec - p_acc - error) / v_target) * hz) * (dt))};
        T p_coast {t * v_target}; 
        error = p_delta_carthesian - p_acc - p_dec - p_coast;

        update_motion (
            static_cast<int> (t * this->hz),
            delta_unit,
            v_target,
            p_acc, 
            true
        );

        // Calculate deceleration phase
        current_motion.calc_constants_v(v_target, v_exit, t_dec);

        update_motion (
            static_cast<int> (t_dec * this->hz),
            delta_unit,
            v_target,
            p_acc + p_coast,
            false
        );
    }

    void update_motion (int n, const std::array<T, N>& unit_vec, T velocity, T p_0, bool is_coast) {
        current_motion.n = n;
        current_motion.dt = dt;
        current_motion.unit_vector = unit_vec;
        current_motion.v_target = velocity;
        current_motion.is_coast = is_coast;
        current_motion.p_0 = p_0;
        current_motion.prev_setpoint = this->mp_buffer[0].setpoint;
        
        this->append_motion(current_motion);

        current_motion.reset();
    }
};

#endif