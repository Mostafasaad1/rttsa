#pragma once

#include "TrajectoryTypes.hpp"
#include <vector>

namespace rt_smoothing {

/**
 * @class SCurvePlanner
 * @brief Real-time trajectory planner for 7-segment S-curve motion profiles
 *
 * This class generates smooth motion profiles with bounded jerk, acceleration,
 * and velocity. It supports single-axis planning, multi-axis synchronization,
 * and mid-motion replanning.
 *
 * Key features:
 * - O(1) deterministic state sampling for real-time control loops
 * - 64-bit double precision for numerical stability
 * - No dynamic memory allocation during execution
 * - Support for mid-motion replanning from non-zero states
 */
class SCurvePlanner {
public:
    SCurvePlanner() = default;
    ~SCurvePlanner() = default;

    /**
     * @brief Plan a trajectory from rest to target position
     * @param distance Target distance to travel
     * @param config Physical constraints (max velocity, acceleration, jerk)
     * @return Complete S-curve profile with 7 segments
     */
    SCurveProfile planTrajectory(
        double distance,
        const TrajectoryConfig& config
    ) const;

    /**
     * @brief Sample the kinematic state at a specific time
     * @param profile Pre-computed trajectory profile
     * @param time Time offset from trajectory start
     * @return Motion state (position, velocity, acceleration) at time t
     *
     * This method executes in O(1) time with no dynamic memory allocation,
     * making it suitable for high-frequency real-time control loops.
     */
    MotionState sampleState(
        const SCurveProfile& profile,
        double time
    ) const;

    /**
     * @brief Plan synchronized trajectories for multiple axes
     * @param distances Target distances for each axis
     * @param configs Physical constraints for each axis
     * @return Vector of synchronized profiles (all finish at same time)
     *
     * Uses time-scaling to ensure all axes complete their movements
     * simultaneously by throttling faster axes.
     */
    std::vector<SCurveProfile> planSynchronizedTrajectory(
        const std::vector<double>& distances,
        const std::vector<TrajectoryConfig>& configs
    );

    /**
     * @brief Plan trajectory from current motion state to target
     * @param target_position Desired final position
     * @param current_state Current kinematic state (position, velocity, acceleration)
     * @param config Physical constraints
     * @return Complete S-curve profile accounting for initial conditions
     *
     * Supports mid-motion replanning without requiring the axis to stop first.
     * Uses closed-form solutions for quadratic and cubic equations when
     * starting from non-zero velocity/acceleration.
     */
    SCurveProfile planFromState(
        double target_position,
        const MotionState& current_state,
        const TrajectoryConfig& config
    );

private:
    void calculateSegmentDurations(
        double distance,
        const TrajectoryConfig& config,
        SCurveProfile& profile
    ) const;

    void precalculateSegmentStates(
        SCurveProfile& profile
    ) const;

    double calculateMaxDuration(
        const std::vector<double>& distances,
        const std::vector<TrajectoryConfig>& configs
    ) const;

    void scaleLimits(
        double target_duration,
        double original_duration,
        TrajectoryConfig& config
    ) const;

    MotionState evaluatePolynomial(
        double time,
        const TrajectorySegment& segment
    ) const;

    /**
     * @brief Solve quadratic equation ax^2 + bx + c = 0
     * @return Largest positive real root, or 0 if no positive solution
     */
    double solveQuadratic(double a, double b, double c) const;

    /**
     * @brief Solve cubic equation ax^3 + bx^2 + cx + d = 0 using Cardano's method
     * @return Largest positive real root, or 0 if no positive solution
     */
    double solveCubic(double a, double b, double c, double d) const;
};

} // namespace rt_smoothing
