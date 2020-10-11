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
 * @file Motion.hpp
 *
 * @brief The motion header holds the final class in the MotionLib hirarchy.
 *
 * @author Bas Brussen
 * Contact: b.brussen@outlook.com
 *
 */

#ifndef Motion_hpp
#define Motion_hpp

#include "MotionPlanner.hpp"

template <typename T, size_t N>
class Motion : public MotionPlanner<T, N> {
public:
    Motion() : MotionPlanner<T, N>(0) {}

    Motion(int hz) : 
        MotionPlanner<T, N>(hz) {}

    Motion(int hz, std::array<T, N> p) : 
        MotionPlanner<T, N>(hz, p),
        p_init(p) { }   

    virtual ~Motion() {}

    inline void plan_motion(std::array<T, N> pos) {
        Point<T, N> p(pos);
        this->plan(p);
    }

    inline void plan_motion(std::array<T, N> pos, T vel, T acc) {
        Point<T, N> p(pos, vel, acc);
        this->plan(p);
    }

    inline void plan_motion(std::array<T, N> pos, T vel, T acc, T v_final) {
        Point<T, N> p(pos, vel, acc);
        this->plan(p, v_final);
    }

    virtual inline void increment_motion_sample() {
        motion_pos++;
    }

    virtual std::array<T, N> get_velocity_setpoint() {
        std::array<T, N> velocities;

        if ((this->motion_queue_size() > 0) && (motion_pos >= current_motion.n)) {
            current_motion = this->get_motion();
            motion_pos = 0;
        }  

        for (size_t i = 0; i < N; i++)
            velocities[i] = current_motion.get_velocity(motion_pos, i);

        return velocities;
    }

    virtual std::array<T, N> get_position_setpoint() {
        std::array<T, N> positions;

        if ((this->motion_queue_size() > 0) && (motion_pos >= current_motion.n)) {
            current_motion = this->get_motion();
            motion_pos = 0;
        }  

        for (size_t i = 0; i < N; i++)
            positions[i] = current_motion.get_position(motion_pos, i) + current_motion.p_prev[i];

        return positions;
    }

    Motion<T, N>& operator= (Motion<T, N>&& mp) {
        this->hz = mp.hz;
        this->dt = mp.dt;
        return *this;
    }

    void set_hz(int hz) {
        this->hz = hz;
    }

private:
    MotionObject<T, N> current_motion;
    std::array<T, N> p_init;
    int motion_pos = 0;

};

#endif