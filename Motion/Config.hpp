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
 * @file Config.hpp
 *
 * @brief The config header is used to define macros that are predefined conditions.
 *
 * @author Bas Brussen
 * Contact: b.brussen@outlook.com
 *
 */

#define CORNER_MAX_RATIO            0.01    // Ratio used to determine exit velocity (v_exit = v_target * CORNER_MAX_RATIO).
#define CORNER_VELOCITY_RATIO       5       // Square factor to determine the ratio (High ratio == slower exit speeds, low ratio == faster exit speeds).

#define STANDARD_FEEDRATE           7200 / 60       // [mm/s]
#define STANDARD_ACCELERATION       2000            // [mm/s^2]