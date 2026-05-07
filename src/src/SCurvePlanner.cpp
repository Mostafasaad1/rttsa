#include "rt_smoothing/SCurvePlanner.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace rt_smoothing {

SCurveProfile SCurvePlanner::planTrajectory(
    double distance,
    const TrajectoryConfig& config
) const {
    if (std::abs(distance) < MIN_DISTANCE_EPSILON) {
        SCurveProfile profile;
        profile.total_duration = 0.0;
    return profile;
    }

    SCurveProfile profile;
    calculateSegmentDurations(distance, config, profile);
    precalculateSegmentStates(profile);

    return profile;
}

MotionState SCurvePlanner::sampleState(
    const SCurveProfile& profile,
    double time
) const {
    if (time <= 0.0) {
        return profile.segments[0].start_state;
    }

    if (time >= profile.total_duration) {
        const auto& last_segment = profile.segments[6];
        return evaluatePolynomial(last_segment.duration, last_segment);
    }

    double accumulated_time = 0.0;
    for (const auto& segment : profile.segments) {
        if (time <= accumulated_time + segment.duration) {
            double segment_time = time - accumulated_time;
            return evaluatePolynomial(segment_time, segment);
        }
        accumulated_time += segment.duration;
    }

    return profile.segments[0].start_state;
}

std::vector<SCurveProfile> SCurvePlanner::planSynchronizedTrajectory(
    const std::vector<double>& distances,
    const std::vector<TrajectoryConfig>& configs
) {
    if (distances.size() != configs.size()) {
        throw std::invalid_argument("distances and configs must have the same size");
    }

    double max_duration = 0.0;
    for (size_t i = 0; i < distances.size(); ++i) {
        SCurveProfile profile = planTrajectory(distances[i], configs[i]);
        max_duration = std::max(max_duration, profile.total_duration);
    }

    std::vector<SCurveProfile> profiles;
    profiles.reserve(distances.size());

    for (size_t i = 0; i < distances.size(); ++i) {
        SCurveProfile temp_profile = planTrajectory(distances[i], configs[i]);

        if (temp_profile.total_duration > 0.0 && std::abs(temp_profile.total_duration - max_duration) > 1e-6) {
            TrajectoryConfig scaled_config = configs[i];

            // Analytic O(1) time-scaling using exact polynomial limit scaling
            double scale_factor = temp_profile.total_duration / max_duration;
            
            scaled_config.max_velocity *= scale_factor;
            scaled_config.max_acceleration *= (scale_factor * scale_factor);
            scaled_config.max_jerk *= (scale_factor * scale_factor * scale_factor);

            temp_profile = planTrajectory(distances[i], scaled_config);
        }

        profiles.push_back(temp_profile);
    }

    return profiles;
}

SCurveProfile SCurvePlanner::planFromState(
    double target_position,
    const MotionState& current_state,
    const TrajectoryConfig& config
) {
    double distance = target_position - current_state.position;
    double v0 = current_state.velocity;
    double a0 = current_state.acceleration;

    if (std::abs(distance) < MIN_DISTANCE_EPSILON &&
        std::abs(v0) < MIN_DISTANCE_EPSILON &&
        std::abs(a0) < MIN_DISTANCE_EPSILON) {
        SCurveProfile profile;
        profile.total_duration = 0.0;
        profile.segments[0].start_state = current_state;
    return profile;
    }

    SCurveProfile profile;
    for (auto& seg : profile.segments) { seg.duration = 0.0; seg.jerk = 0.0; }

    if (std::abs(v0) < MIN_DISTANCE_EPSILON && std::abs(a0) < MIN_DISTANCE_EPSILON) {
        calculateSegmentDurations(distance, config, profile);
        profile.segments[0].start_state = current_state;
        precalculateSegmentStates(profile);
    return profile;
    }

    double dir = distance >= 0.0 ? 1.0 : -1.0;
    double J = config.max_jerk * dir;
    double A = config.max_acceleration;
    double V = config.max_velocity;

    // Calculate exact stopping distance from v0, a0
    double J_stop = (v0 > 0) ? std::abs(config.max_jerk) : -std::abs(config.max_jerk);
    if (std::abs(v0) < MIN_DISTANCE_EPSILON) {
        J_stop = (a0 > 0) ? std::abs(config.max_jerk) : -std::abs(config.max_jerk);
    }

    double qA = -0.5 * J_stop;
    double qB = a0;
    double qC = v0 - (a0 * a0) / (2.0 * J_stop);
    
    double t1_stop = solveQuadratic(qA, qB, qC);
    if (t1_stop < 0.0 || std::isnan(t1_stop)) t1_stop = 0.0;
    double t2_stop = t1_stop - a0 / J_stop;
    if (t2_stop < 0.0 || std::isnan(t2_stop)) t2_stop = 0.0;
    
    TrajectorySegment s1(t1_stop, -J_stop, current_state);
    MotionState mid = evaluatePolynomial(t1_stop, s1);
    TrajectorySegment s2(t2_stop, J_stop, mid);
    MotionState stop_state = evaluatePolynomial(t2_stop, s2);
    
    bool overshoots = false;
    if (dir > 0 && stop_state.position > target_position) overshoots = true;
    if (dir < 0 && stop_state.position < target_position) overshoots = true;

    if (overshoots) {
        profile.segments[0].duration = t1_stop;
        profile.segments[0].jerk = -J_stop;
        profile.segments[1].duration = t2_stop;
        profile.segments[1].jerk = J_stop;
        
        double remaining_dist = target_position - stop_state.position;
        SCurveProfile rem_profile;
        calculateSegmentDurations(remaining_dist, config, rem_profile);
        
        int idx = 2;
        for (int i = 0; i < 7; ++i) {
            if (rem_profile.segments[i].duration > 1e-6 && idx < 7) {
                profile.segments[idx].duration = rem_profile.segments[i].duration;
                profile.segments[idx].jerk = rem_profile.segments[i].jerk;
                idx++;
            }
        }
    } else {
        double abs_v0 = std::abs(v0);
        double abs_J = std::abs(J);
        
        double tj1 = A / abs_J;
        double dv_j1 = abs_J * tj1 * tj1;
        double ta1 = 0.0;
        if (V - abs_v0 >= dv_j1) {
            ta1 = (V - abs_v0 - dv_j1) / A;
        } else {
            tj1 = std::sqrt(std::abs(V - abs_v0) / abs_J);
        }
        double d_accel = abs_v0 * (2 * tj1 + ta1) + 0.5 * (V - abs_v0) * (2 * tj1 + ta1);

        double tj2 = A / abs_J;
        double dv_j2 = abs_J * tj2 * tj2;
        double ta2 = 0.0;
        if (V >= dv_j2) {
            ta2 = (V - dv_j2) / A;
        } else {
            tj2 = std::sqrt(V / abs_J);
        }
        double d_decel = 0.5 * V * (2 * tj2 + ta2);

        double d_reach = d_accel + d_decel;
        
        double tv = 0.0;
        if (std::abs(distance) >= d_reach) {
            tv = (std::abs(distance) - d_reach) / V;
            
            profile.segments[0].duration = tj1; profile.segments[0].jerk = J;
            profile.segments[1].duration = ta1; profile.segments[1].jerk = 0.0;
            profile.segments[2].duration = tj1; profile.segments[2].jerk = -J;
            profile.segments[3].duration = tv;  profile.segments[3].jerk = 0.0;
            profile.segments[4].duration = tj2; profile.segments[4].jerk = -J;
            profile.segments[5].duration = ta2; profile.segments[5].jerk = 0.0;
            profile.segments[6].duration = tj2; profile.segments[6].jerk = J;
        } else {
            double t_cubic = solveCubic(abs_J / 6.0, std::abs(a0) / 2.0, abs_v0, -std::abs(distance));
            if (t_cubic > 0.0) {
                double t_seg = t_cubic / 4.0;
                profile.segments[0].duration = t_seg; profile.segments[0].jerk = J;
                profile.segments[2].duration = t_seg; profile.segments[2].jerk = -J;
                profile.segments[4].duration = t_seg; profile.segments[4].jerk = -J;
                profile.segments[6].duration = t_seg; profile.segments[6].jerk = J;
            }
        }
    }

    profile.total_duration = 0.0;
    for (int i = 0; i < 7; ++i) {
        profile.total_duration += profile.segments[i].duration;
    }

    profile.segments[0].start_state = current_state;
    precalculateSegmentStates(profile);

    return profile;
}

void SCurvePlanner::calculateSegmentDurations(
    double distance,
    const TrajectoryConfig& config,
    SCurveProfile& profile
) const {
    double abs_distance = std::abs(distance);
    double dir = distance >= 0.0 ? 1.0 : -1.0;

    for (auto& seg : profile.segments) { seg.duration = 0.0; seg.jerk = 0.0; }
    profile.total_duration = 0.0;

    if (abs_distance < MIN_DISTANCE_EPSILON) return;

    double J = config.max_jerk;
    double A = config.max_acceleration;
    double V = config.max_velocity;

    double tj_max = A / J;
    double vj_max = J * tj_max * tj_max; 
    
    double t_j = 0, t_a = 0, t_v = 0;

    if (V >= vj_max) {
        t_a = (V - vj_max) / A;
        t_j = tj_max;
    } else {
        t_j = std::sqrt(V / J);
        t_a = 0.0;
    }

    double d_accel = V * (t_j + 0.5 * t_a);
    double d_reach_max = 2.0 * d_accel;

    if (abs_distance >= d_reach_max) {
        t_v = (abs_distance - d_reach_max) / V;
    } else {
        t_v = 0.0;
        t_j = tj_max;
        double a_coeff = 0.5 * A;
        double b_coeff = 1.5 * A * A / J;
        double c_coeff = (A * A * A) / (J * J) - abs_distance / 2.0;
        
        double t_a_candidate = solveQuadratic(a_coeff, b_coeff, c_coeff);
        
        if (t_a_candidate >= 0.0 && c_coeff <= 0.0) {
            t_a = t_a_candidate;
        } else {
            t_a = 0.0;
            // Solve cubic implicitly: D = 2 * J * t_j^3 -> t_j = cbrt(D / 2J)
            t_j = std::cbrt(abs_distance / (2.0 * J));
        }
    }

    profile.segments[0].duration = t_j; profile.segments[0].jerk = dir * J;
    profile.segments[1].duration = t_a; profile.segments[1].jerk = 0.0;
    profile.segments[2].duration = t_j; profile.segments[2].jerk = -dir * J;
    
    profile.segments[3].duration = t_v; profile.segments[3].jerk = 0.0;
    
    profile.segments[4].duration = t_j; profile.segments[4].jerk = -dir * J;
    profile.segments[5].duration = t_a; profile.segments[5].jerk = 0.0;
    profile.segments[6].duration = t_j; profile.segments[6].jerk = dir * J;

    profile.total_duration = 4.0 * t_j + 2.0 * t_a + t_v;
}

void SCurvePlanner::precalculateSegmentStates(
    SCurveProfile& profile
) const {
    MotionState current_state = profile.segments[0].start_state;

    for (size_t i = 0; i < profile.segments.size(); ++i) {
        profile.segments[i].start_state = current_state;

        const auto& segment = profile.segments[i];
        double t = segment.duration;
        double j = segment.jerk;

        double a0 = current_state.acceleration;
        double v0 = current_state.velocity;
        double p0 = current_state.position;

        current_state.acceleration = a0 + j * t;
        current_state.velocity = v0 + a0 * t + 0.5 * j * t * t;
        current_state.position = p0 + v0 * t + 0.5 * a0 * t * t + (1.0 / 6.0) * j * t * t * t;
    }
}

double SCurvePlanner::calculateMaxDuration(
    const std::vector<double>& distances,
    const std::vector<TrajectoryConfig>& configs
) const {
    double max_duration = 0.0;

    for (size_t i = 0; i < distances.size(); ++i) {
        SCurveProfile profile = planTrajectory(distances[i], configs[i]);
        max_duration = std::max(max_duration, profile.total_duration);
    }

    return max_duration;
}

void SCurvePlanner::scaleLimits(
    double target_duration,
    double original_duration,
    TrajectoryConfig& config
) const {
    if (original_duration <= 0.0) return;

    double scale_factor = original_duration / target_duration;
    if (scale_factor <= 0.0) return;

    config.max_velocity /= scale_factor;
    config.max_acceleration /= scale_factor;
    config.max_jerk /= scale_factor;
}

MotionState SCurvePlanner::evaluatePolynomial(
    double time,
    const TrajectorySegment& segment
) const {
    MotionState state;
    double j = segment.jerk;
    double t = time;

    double a0 = segment.start_state.acceleration;
    double v0 = segment.start_state.velocity;
    double p0 = segment.start_state.position;

    state.acceleration = a0 + j * t;
    state.velocity = v0 + a0 * t + 0.5 * j * t * t;
    state.position = p0 + v0 * t + 0.5 * a0 * t * t + (1.0 / 6.0) * j * t * t * t;

    return state;
}

double SCurvePlanner::solveQuadratic(double a, double b, double c) const {
    double discriminant = b * b - 4.0 * a * c;

    if (discriminant < 0.0) {
        return 0.0;
    }

    double sqrt_discriminant = std::sqrt(discriminant);
    double x1 = (-b + sqrt_discriminant) / (2.0 * a);
    double x2 = (-b - sqrt_discriminant) / (2.0 * a);

    return std::max(x1, x2);
}

double SCurvePlanner::solveCubic(double a, double b, double c, double d) const {
    if (std::abs(a) < MIN_DISTANCE_EPSILON) {
        return solveQuadratic(b, c, d);
    }

    double p = (3.0 * a * c - b * b) / (3.0 * a * a);
    double q = (2.0 * b * b * b - 9.0 * a * b * c + 27.0 * a * a * d) / (27.0 * a * a * a);

    double discriminant = q * q / 4.0 + p * p * p / 27.0;

    if (discriminant > 0.0) {
        double sqrt_discriminant = std::sqrt(discriminant);
        double u = std::cbrt(-q / 2.0 + sqrt_discriminant);
        double v = std::cbrt(-q / 2.0 - sqrt_discriminant);
        return u + v - b / (3.0 * a);
    } else if (discriminant == 0.0) {
        double u = std::cbrt(-q / 2.0);
        return 2.0 * u - b / (3.0 * a);
    } else {
        double r = std::sqrt(-p * p * p / 27.0);
        double theta = std::acos(-q / (2.0 * r));
        double x1 = 2.0 * std::cbrt(r) * std::cos(theta / 3.0) - b / (3.0 * a);
        double x2 = 2.0 * std::cbrt(r) * std::cos((theta + 2.0 * M_PI) / 3.0) - b / (3.0 * a);
        double x3 = 2.0 * std::cbrt(r) * std::cos((theta + 4.0 * M_PI) / 3.0) - b / (3.0 * a);

        double max_x = std::max({x1, x2, x3});
        return max_x > 0.0 ? max_x : 0.0;
    }
}

} // namespace rt_smoothing
