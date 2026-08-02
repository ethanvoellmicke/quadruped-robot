#include <algorithm>
#include <cmath>
#include <iostream>

#include "qr_robot_control_cpp/leg_ik.hpp"

namespace
{
constexpr double UPPER_LEG_LENGTH = 0.110;
constexpr double LOWER_LEG_LENGTH = 0.1105;
}

LegAngles solveLegIK(double x, double z)
{
    const double requested_distance =
        std::sqrt(x * x + z * z);

    const double maximum_reach =
        UPPER_LEG_LENGTH + LOWER_LEG_LENGTH - 0.001;

    const double minimum_reach =
        std::abs(UPPER_LEG_LENGTH - LOWER_LEG_LENGTH) + 0.001;

    const double distance =
        std::clamp(
            requested_distance,
            minimum_reach,
            maximum_reach
        );
/*
    if (requested_distance > maximum_reach)
    {
        std::cout
            << "Workspace limit hit!"
            << "  x = " << x
            << "  z = " << z
            << "  requested = " << requested_distance
            << "  max = " << maximum_reach
            << std::endl;
    }
*/
    double cosine_knee =
        (
            distance * distance
            - UPPER_LEG_LENGTH * UPPER_LEG_LENGTH
            - LOWER_LEG_LENGTH * LOWER_LEG_LENGTH
        )
        /
        (
            2.0
            * UPPER_LEG_LENGTH
            * LOWER_LEG_LENGTH
        );

    cosine_knee =
        std::clamp(cosine_knee, -1.0, 1.0);

    const double knee =
        -std::acos(cosine_knee);

    const double angle_to_foot =
        std::atan2(z, x);

    const double internal_angle =
        std::atan2(
            LOWER_LEG_LENGTH * std::sin(std::abs(knee)),
            UPPER_LEG_LENGTH
            + LOWER_LEG_LENGTH * std::cos(std::abs(knee))
        );

    const double hip =
        angle_to_foot + internal_angle;

    return {hip, knee};
}