#include "Motion.hpp"

template <typename T>
class Orientation : Motion<T, 4>{
public:
    Orientation(int hz) : Motion<T, 4>(hz) { }

    void set_orientation(std::array<T, 3>& orientation) {
        auto q = ToQuaternion(orientation);
        Point<T, 4> p(q.w, q.x, q.y, q.z);
        this->do_command(p);
    }

    std::array<T, 3> get_angular_velocity_setpoint() {
        std::array<T, 4> quat;

        if ((this->motion_queue_size() > 0) && (this->motion_pos >= current_motion.n)) {
            current_motion = this->get_motion();
            this->motion_pos = 0;
        }  

        for (size_t i = 0; i < 4; i++)
            quat[i] = current_motion.get_velocity(this->motion_pos, i);

        return ToEulerAngles(quat);
    }

    std::array<T, 3> get_angular_position_setpoint() {
        std::array<T, 4> quat;

        if ((this->motion_queue_size() > 0) && (this->motion_pos >= current_motion.n)) {
            current_motion = this->get_motion();
            this->motion_pos = 0;
        }  

        for (size_t i = 0; i < 4; i++)
            quat[i] = current_motion.get_position(this->motion_pos, i);

        return ToEulerAngles(quat);
    }

private:
    struct Quaternion {
    std::array<T, 4> ToQuaternion(std::array<T, 3>& e) {
        // Abbreviations for the various angular functions
        T cy = cos(e[2] * 0.5);
        T sy = sin(e[2] * 0.5);
        T cp = cos(e[1] * 0.5);
        T sp = sin(e[1] * 0.5);
        T cr = cos(e[0] * 0.5);
        T sr = sin(e[0] * 0.5);

        std::array<T, 4> q;

        q[0] = cr * cp * cy + sr * sp * sy;
        q[1] = sr * cp * cy - cr * sp * sy;
        q[2] = cr * sp * cy + sr * cp * sy;
        q[3] = cr * cp * sy - sr * sp * cy;

        return q;
    }

    std::array<T, 3> ToEulerAngles(std::array<T, 4>& q) {
        std::array<T, 3> e;

        // roll (x-axis rotation)
        T sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
        T cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
        e[0] = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        T sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
        if (std::abs(sinp) >= 1)
            e[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            e[1] = std::asin(sinp);

        // yaw (z-axis rotation)
        T siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
        T cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
        e[2] = std::atan2(siny_cosp, cosy_cosp);

        return e;
    }
};