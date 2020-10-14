

#ifndef MotionBuffer_hpp
#define MotionBuffer_hpp

#include "Definitions.hpp"

template <typename T, size_t N>
class MotionBuffer {
public:
    MotionBuffer () { }

    MotionBuffer (std::array<T, N>& p_init) {
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