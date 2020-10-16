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
 * @file SetpointBuffer.hpp
 *
 * @brief The setpoint buffer is responsible to buffer up to three user defined setpoints.
 *
 * @author Bas Brussen
 * Contact: b.brussen@outlook.com
 *
 */

#ifndef SetpointBuffer_hpp
#define SetpointBuffer_hpp

#include "Definitions.hpp"

template <typename T, size_t N>
class SetpointBuffer {
public:
    SetpointBuffer () { }

    SetpointBuffer (std::array<T, N>& p_init) {
        append_buffer(p_init);
        append_buffer(p_init);
        append_buffer(p_init);
    }

protected:
    void append_buffer(const Point<T, N>& p){
        mp_buffer[0] = mp_buffer[1];
        mp_buffer[1] = mp_buffer[2]; 
        mp_buffer[2] = std::move(p);
    }

    std::array<Point<T, N>, 3> mp_buffer;
};

#endif