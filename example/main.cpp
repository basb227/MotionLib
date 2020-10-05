#include <iostream>

// Only include Motion.hpp
#include "../Motion/Motion.hpp"

int main () {
    // Class Motion takes two template arguments: first is type (float, double, int etc.), 
    // second is number of dimensions.
    // The object takes one argument: hz, which is the amount of samples created per second (or resolution).
    Motion<double, 3> motion(1000);

    // set_position takes array which represents desired position.
    // Positions are always absolute.
    std::array<double, 3> p {4, 5, 6};

    // Give desired position with velocity target and max acceleration.
    // The 4yh parameter is user specified exit velocity (set 0 when motion should stop)
    // or do not enter to let the planner calculate exit velocity.
    motion.set_position(p, 100, 500, 0);

    for (int i = 0; i < 748; i++) {
        // Get velocity setpoints of current time sample.
        auto result = motion.get_velocity_setpoint();

        // Increment motion.
        motion.increment_motion_sample();

        std::cout   << result[0] << ", " 
                    << result[1] << ", " 
                    << result[2] << "\n";
    }
    
    return 0;
}