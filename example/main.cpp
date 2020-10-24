#include <iostream>

// Only include Motion.hpp
#include "../Motion/Motion.hpp"

int main() {
	// Class Motion takes two template arguments: first is type (float, double, int etc.), 
	// second is number of dimensions.
	// The object takes one argument: hz, which is the amount of samples created per second (or resolution).
	Motion<double, 6> motion(1000);

	// set_position takes array which represents desired position.
	// Positions are always absolute.
	std::array<double, 6> p{ 1, 2, 3, 4, 5, 6 };

	// Orientation and position are given in three dimensions.
	// The motion class will calculate for the 6 dimensions.


	// Plan the motion. The paramters are velocity acceleration and final velocity respectively.
	motion.plan_motion(p, 500, 1000, 250);
	motion.plan_motion(p, 500, 1000, 250);

	std::cout << "roll, pitch, yaw, x, y, z" << "\n";

	bool motion_state = true;
	while (motion_state) {
		// Get velocity setpoints of current time sample.
		auto result = motion.get_velocity_setpoint();

		// Increment motion.
		// Return false when all motion samples are returned.
		motion_state = motion.increment_motion_sample();

		std::cout << result[0] << ", "
			<< result[1] << ", "
			<< result[2] << ", "
			<< result[3] << ", "
			<< result[4] << ", "
			<< result[5] << "\n";
	}

	return 0;
}