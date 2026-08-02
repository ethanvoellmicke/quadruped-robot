#ifndef QR_ROBOT_CONTROL_CPP__FOOT_TRAJECTORY_HPP_
#define QR_ROBOT_CONTROL_CPP__FOOT_TRAJECTORY_HPP_

#include "qr_robot_control_cpp/gait_types.hpp"

FootPosition calculateStepTrajectory(
    double time,
    double phase_offset,
    double speed,
    double z_ground,//defined in gait controller
    double x_center = 0.04,
    double step_length = 0.12,//.025
    double step_height = 0.05,//.06
    double stance_depth = 0.008
);

#endif