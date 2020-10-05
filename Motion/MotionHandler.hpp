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
 * @file MotionHandler.hpp
 *
 * @brief The motion handler header holds the class which queues and returns calculated motions.
 *
 * @author Bas Brussen
 * Contact: b.brussen@outlook.com
 *
 */

#include "Utils.hpp"
#include <queue>

template <typename T, size_t N>
class MotionHandler{
public:
    MotionHandler () {}

    virtual ~MotionHandler() {}

    void append_motion (MotionObject<T, N>& m) {
        motion_queue.push(m);
    }

    int motion_queue_size () {
        return motion_queue.size();
    }

    MotionObject<T, N> get_motion () {
        if (motion_queue.size() > 0){
            MotionObject<T, N> move = std::move(motion_queue.front());
            motion_queue.pop();
            return move;
        }

        return MotionObject<T, N>();
    }

private:
    std::queue<MotionObject<T, N>> motion_queue;
};