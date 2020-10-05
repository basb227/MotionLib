#include <iostream>

#include "../Motion/Motion.hpp"

int main () {
    Motion<double, 3> motion(1000);

    std::array<double, 3> p {4, 5, 6};
    motion.set_position(p, 100, 500);

    for (int i = 0; i < 375; i++) {
        auto result = motion.get_next_velocity_setpoint();
        std::cout << result[0] << ", " << result[1] << ", " << result[2] << "\n";
    }
    
    return 0;
}