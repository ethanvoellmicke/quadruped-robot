#include <algorithm>
#include <cmath>

#include "qr_robot_control_cpp/foot_trajectory.hpp"

namespace
{
constexpr double PI = 3.14159265358979323846;

/*
 * Rises quickly at the beginning, then slows as it reaches the top.
 *
 * t = 0 -> 0
 * t = 1 -> 1
 */
double easeOutCubic(double t)
{
    t = std::clamp(t, 0.0, 1.0);

    const double one_minus_t = 1.0 - t;

    return 1.0
        - one_minus_t
        * one_minus_t
        * one_minus_t;
}

/*
 * Starts slowly and accelerates near the end.
 *
 * Useful for lowering the foot without immediately dropping it.
 */
double easeInCubic(double t)
{
    t = std::clamp(t, 0.0, 1.0);

    return t * t * t;
}

/*
 * Smooth motion from 0 to 1.
 */
double smoothStep(double t)
{
    t = std::clamp(t, 0.0, 1.0);

    return t * t * (3.0 - 2.0 * t);
}
}

FootPosition calculateStepTrajectory(
    double time,
    double phase_offset,
    double speed,
    double z_ground,
    double x_center,
    double step_length,
    double step_height,
    double stance_depth
)
{
    double phase =
        std::fmod(
            -speed * time + phase_offset,
            2.0 * PI
        );

    if (phase < 0.0)
    {
        phase += 2.0 * PI;
    }

    // Convert phase from 0 -> 2π into cycle progress from 0 -> 1.
    const double cycle_t =
        phase / (2.0 * PI);

    const double x_back =
        x_center - 0.5 * step_length;

    const double x_front =
        x_center + 0.5 * step_length;

    FootPosition foot{};

    /*
     * 25% of the cycle is swing.
     * 75% of the cycle is stance.
     *
     * With four legs separated by 25% of a cycle,
     * this allows approximately one leg to swing at a time.
     */
    constexpr double SWING_PORTION = 0.25;

    if (cycle_t < SWING_PORTION)
    {
        /*
         * Normalize the shortened swing interval back to 0 -> 1.
         */
        const double swing_t =
            cycle_t / SWING_PORTION;

        constexpr double LIFT_END = 0.30;
        constexpr double DROP_START = 0.70;

        // Part 1: lift almost vertically.
        if (swing_t < LIFT_END)
        {
            const double t =
                swing_t / LIFT_END;

            const double lift =
                easeOutCubic(t);

            foot.x = x_back;

            foot.z =
                z_ground
                + step_height * lift;
        }

        // Part 2: move forward while lifted.
        else if (swing_t < DROP_START)
        {
            const double t =
                (swing_t - LIFT_END)
                / (DROP_START - LIFT_END);

            const double forward =
                smoothStep(t);

            foot.x =
                x_back
                + step_length * forward;

            foot.z =
                z_ground
                + step_height;
        }

        // Part 3: lower at the front.
        else
        {
            const double t =
                (swing_t - DROP_START)
                / (1.0 - DROP_START);

            const double drop =
                easeInCubic(t);

            foot.x = x_front;

            foot.z =
                z_ground
                + step_height * (1.0 - drop);
        }
    }
    else
    {
        /*
         * Normalize the 75% stance interval back to 0 -> 1.
         */
        const double stance_t =
            (cycle_t - SWING_PORTION)
            / (1.0 - SWING_PORTION);

        foot.x =
            x_front
            - step_length * stance_t;

        foot.z =
            z_ground
            - stance_depth
            * std::sin(PI * stance_t);
    }

    return foot;
}