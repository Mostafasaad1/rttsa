#pragma once

#include <array>
#include <cstdint>

namespace rt_smoothing {

constexpr double MIN_DISTANCE_EPSILON = 1e-9;

/**
 * @struct TrajectoryConfig
 * @brief Physical constraints for a motor axis
 *
 * Defines the maximum kinematic limits that the trajectory planner
 * must respect when generating motion profiles.
 */
struct TrajectoryConfig {
    double max_velocity;      ///< Maximum allowed velocity (units/s)
    double max_acceleration;  ///< Maximum allowed acceleration (units/s²)
    double max_jerk;          ///< Maximum allowed jerk (units/s³)

    TrajectoryConfig(double v, double a, double j)
        : max_velocity(v), max_acceleration(a), max_jerk(j) {}
};

/**
 * @struct MotionState
 * @brief Instantaneous kinematic state of an axis
 *
 * Represents the complete state of motion at a specific point in time.
 * Used for both initial conditions and sampled states during trajectory execution.
 */
struct MotionState {
    double position;       ///< Current position (units)
    double velocity;       ///< Current velocity (units/s)
    double acceleration;  ///< Current acceleration (units/s²)

    MotionState(double p = 0.0, double v = 0.0, double a = 0.0)
        : position(p), velocity(v), acceleration(a) {}
};

/**
 * @struct TrajectorySegment
 * @brief Single phase of a 7-segment S-curve
 *
 * Each segment has constant jerk and pre-calculated initial conditions,
 * enabling O(1) state evaluation during real-time execution.
 */
struct TrajectorySegment {
    double duration;      ///< Time spent in this segment (seconds)
    double jerk;          ///< Constant jerk applied during this segment (units/s³)
    MotionState start_state;  ///< Kinematic state at segment start

    TrajectorySegment(double d = 0.0, double j = 0.0, const MotionState& s = MotionState())
        : duration(d), jerk(j), start_state(s) {}
};

/**
 * @struct SCurveProfile
 * @brief Complete 7-segment S-curve motion profile
 *
 * Contains all segment data required to sample the motion at any time.
 * The profile is generated once (non-real-time) and sampled many times
 * (real-time) during trajectory execution.
 */
struct SCurveProfile {
    std::array<TrajectorySegment, 7> segments;  ///< The 7 kinematic phases
    double total_duration;  ///< Total time for complete trajectory (seconds)

    SCurveProfile() : total_duration(0.0) {
        segments.fill(TrajectorySegment());
    }
};

static_assert(sizeof(double) == 8, "64-bit double precision required");

} // namespace rt_smoothing
