

#ifndef MotionBuffer_hpp
#define MotionBuffer_hpp

#include "Definitions.hpp"

template <typename T, size_t N>
class MotionBuffer {
public:
    std::array<T, N> p_init;

    MotionBuffer () {
        Point<T, N> p(STANDARD_VELOCITY, STANDARD_ACCELERATION);

        append_buffer(p);
        append_buffer(p);
        append_buffer(p);
    }

    MotionBuffer (std::array<T, N> p_init) :
        p_init(p_init) {
            append_buffer(p_init);
            append_buffer(p_init);
            append_buffer(p_init);
    }

    void append_buffer(const Point<T, N>& p){
        mp_buffer[0] = mp_buffer[1];
        mp_buffer[1] = mp_buffer[2]; 
        mp_buffer[2] = std::move(p);
    }

private:
    std::array<Point<T, N>, 3> mp_buffer;
    bool is_well_defined = true;

    void check_definition(){
        is_well_defined =   mp_buffer[0] == mp_buffer[1] ? 
                            mp_buffer[1] == mp_buffer[2] ? true : false;
    }
};

#endif