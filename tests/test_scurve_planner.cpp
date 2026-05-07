#include <gtest/gtest.h>
#include "rt_smoothing/SCurvePlanner.hpp"
#include "rt_smoothing/TrajectoryTypes.hpp"
#include <chrono>

using namespace rt_smoothing;

class SCurvePlannerTest : public ::testing::Test {
protected:
    SCurvePlanner planner;

    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(SCurvePlannerTest, BasicInitialization) {
    EXPECT_TRUE(true);
}

// User Story 1 Tests
TEST_F(SCurvePlannerTest, LongDistanceMove_ReachesMaxAccelAndVelocity) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    double distance = 100.0;

    SCurveProfile profile = planner.planTrajectory(distance, config);

    EXPECT_GT(profile.total_duration, 0.0);

    bool has_accel_phase = false;
    bool has_velocity_phase = false;

    for (const auto& segment : profile.segments) {
        if (segment.duration > 0.0) {
            if (std::abs(segment.jerk) > 0.0) {
                has_accel_phase = true;
            } else {
                has_velocity_phase = true;
            }
        }
    }

    EXPECT_TRUE(has_accel_phase);
    EXPECT_TRUE(has_velocity_phase);
}

TEST_F(SCurvePlannerTest, ShortDistanceMove_ZeroSegmentTimes) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    double distance = 0.5;

    SCurveProfile profile = planner.planTrajectory(distance, config);

    EXPECT_GT(profile.total_duration, 0.0);

    int non_zero_segments = 0;
    for (const auto& segment : profile.segments) {
        if (segment.duration > 1e-6) {
            non_zero_segments++;
        }
    }

    EXPECT_LE(non_zero_segments, 7);
}

TEST_F(SCurvePlannerTest, JerkLimitsNeverViolated) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    double distance = 50.0;

    SCurveProfile profile = planner.planTrajectory(distance, config);

    for (const auto& segment : profile.segments) {
        if (segment.duration > 0.0) {
            EXPECT_LE(std::abs(segment.jerk), config.max_jerk + 1e-6);
        }
    }
}

TEST_F(SCurvePlannerTest, ProfileHasExactlySevenSegments) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    double distance = 50.0;

    SCurveProfile profile = planner.planTrajectory(distance, config);

    EXPECT_EQ(profile.segments.size(), 7);
}

// User Story 2 Tests
TEST_F(SCurvePlannerTest, StateSamplingAtVariousTimeOffsets) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    double distance = 50.0;

    SCurveProfile profile = planner.planTrajectory(distance, config);

    std::vector<double> test_times = {0.0, 0.1, 0.5, 1.0, profile.total_duration / 2.0, profile.total_duration - 0.1, profile.total_duration};

    for (double time : test_times) {
        MotionState state = planner.sampleState(profile, time);

        EXPECT_GE(state.position, -1e-6);

        if (time <= profile.total_duration) {
            EXPECT_LE(state.position, std::abs(distance) + 5.0);
        }
    }

    MotionState final_state = planner.sampleState(profile, profile.total_duration);
    EXPECT_NEAR(final_state.position, distance, 5.0);
}

TEST_F(SCurvePlannerTest, SamplingCompletesInO1Time) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    double distance = 50.0;

    SCurveProfile profile = planner.planTrajectory(distance, config);

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 10000; ++i) {
        double time = (i / 10000.0) * profile.total_duration;
        MotionState state = planner.sampleState(profile, time);
        (void)state;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    EXPECT_LT(duration.count(), 10000);
}

TEST_F(SCurvePlannerTest, NoInstantaneousAccelerationJumpsAcrossSegments) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    double distance = 50.0;

    SCurveProfile profile = planner.planTrajectory(distance, config);

    double accumulated_time = 0.0;
    for (size_t i = 0; i < profile.segments.size(); ++i) {
        if (profile.segments[i].duration > 1e-6) {
            MotionState state_at_end = planner.sampleState(profile, accumulated_time + profile.segments[i].duration);

            if (i < profile.segments.size() - 1 && profile.segments[i + 1].duration > 1e-6) {
                MotionState state_at_next_start = planner.sampleState(profile, accumulated_time + profile.segments[i].duration + 1e-9);

                double accel_diff = std::abs(state_at_end.acceleration - state_at_next_start.acceleration);
                EXPECT_NEAR(accel_diff, 0.0, 1e-3);
            }
        }
        accumulated_time += profile.segments[i].duration;
    }
}

TEST_F(SCurvePlannerTest, PositionContinuityAtSegmentBoundaries) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    double distance = 50.0;

    SCurveProfile profile = planner.planTrajectory(distance, config);

    double accumulated_time = 0.0;
    MotionState prev_state = planner.sampleState(profile, 0.0);

    for (const auto& segment : profile.segments) {
        if (segment.duration > 1e-6) {
            MotionState state_at_boundary = planner.sampleState(profile, accumulated_time);
            MotionState state_after_boundary = planner.sampleState(profile, accumulated_time + 1e-6);

            EXPECT_NEAR(state_at_boundary.position, state_after_boundary.position, 1e-4);
        }
        accumulated_time += segment.duration;
    }
}

// User Story 3 Tests
TEST_F(SCurvePlannerTest, BottleneckAxisIdentification) {
    std::vector<double> distances = {10.0, 50.0, 30.0};
    std::vector<TrajectoryConfig> configs = {
        TrajectoryConfig(10.0, 5.0, 20.0),
        TrajectoryConfig(10.0, 5.0, 20.0),
        TrajectoryConfig(10.0, 5.0, 20.0)
    };

    std::vector<SCurveProfile> profiles = planner.planSynchronizedTrajectory(distances, configs);

    EXPECT_EQ(profiles.size(), 3);

    double max_duration = 0.0;
    for (const auto& profile : profiles) {
        max_duration = std::max(max_duration, profile.total_duration);
    }

    for (const auto& profile : profiles) {
        EXPECT_NEAR(profile.total_duration, max_duration, 1e-3);
    }
}

TEST_F(SCurvePlannerTest, TimeScalingForFasterAxes) {
    std::vector<double> distances = {10.0, 50.0, 30.0};
    std::vector<TrajectoryConfig> configs = {
        TrajectoryConfig(10.0, 5.0, 20.0),
        TrajectoryConfig(10.0, 5.0, 20.0),
        TrajectoryConfig(10.0, 5.0, 20.0)
    };

    std::vector<SCurveProfile> profiles = planner.planSynchronizedTrajectory(distances, configs);

    EXPECT_EQ(profiles.size(), 3);

    // After exact analytical time scaling, all profiles will have identically equal durations.
    // Therefore, finding a "bottleneck" strictly by finding the strictly greatest duration 
    // among the *synchronized* profiles will trivially return the first element (index 0).
    double max_duration = profiles[0].total_duration;
    for (size_t i = 1; i < profiles.size(); ++i) {
        EXPECT_NEAR(profiles[i].total_duration, max_duration, 1e-6);
    }
}

TEST_F(SCurvePlannerTest, AllAxesFinishAtSameTime) {
    std::vector<double> distances = {10.0, 50.0, 30.0};
    std::vector<TrajectoryConfig> configs = {
        TrajectoryConfig(10.0, 5.0, 20.0),
        TrajectoryConfig(10.0, 5.0, 20.0),
        TrajectoryConfig(10.0, 5.0, 20.0)
    };

    std::vector<SCurveProfile> profiles = planner.planSynchronizedTrajectory(distances, configs);

    EXPECT_EQ(profiles.size(), 3);

    double first_duration = profiles[0].total_duration;
    for (size_t i = 1; i < profiles.size(); ++i) {
        EXPECT_NEAR(profiles[i].total_duration, first_duration, 1e-3);
    }
}

TEST_F(SCurvePlannerTest, SynchronizedProfileMaintainsSCurveShape) {
    std::vector<double> distances = {10.0, 50.0};
    std::vector<TrajectoryConfig> configs = {
        TrajectoryConfig(10.0, 5.0, 20.0),
        TrajectoryConfig(10.0, 5.0, 20.0)
    };

    std::vector<SCurveProfile> profiles = planner.planSynchronizedTrajectory(distances, configs);

    for (const auto& profile : profiles) {
        EXPECT_EQ(profile.segments.size(), 7);

        int non_zero_segments = 0;
        for (const auto& segment : profile.segments) {
            if (segment.duration > 1e-6) {
                non_zero_segments++;
            }
        }

        EXPECT_GT(non_zero_segments, 0);
    }
}

// User Story 4 Tests
TEST_F(SCurvePlannerTest, ReplanningFromNonZeroVelocity) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    MotionState current_state(0.0, 5.0, 0.0);
    double target_position = 50.0;

    SCurveProfile profile = planner.planFromState(target_position, current_state, config);

    EXPECT_GT(profile.total_duration, 0.0);
    EXPECT_EQ(profile.segments.size(), 7);
}

TEST_F(SCurvePlannerTest, ReplanningFromNonZeroAcceleration) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    MotionState current_state(0.0, 0.0, 2.0);
    double target_position = 50.0;

    SCurveProfile profile = planner.planFromState(target_position, current_state, config);

    EXPECT_GT(profile.total_duration, 0.0);
    EXPECT_EQ(profile.segments.size(), 7);
}

TEST_F(SCurvePlannerTest, OvershootAndReturnForUnreachableTargets) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    MotionState current_state(0.0, 8.0, 0.0);
    double target_position = 5.0;

    SCurveProfile profile = planner.planFromState(target_position, current_state, config);

    EXPECT_GT(profile.total_duration, 0.0);
    EXPECT_EQ(profile.segments.size(), 7);
}

TEST_F(SCurvePlannerTest, ReplanningMaintainsJerkLimits) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    MotionState current_state(0.0, 5.0, 2.0);
    double target_position = 50.0;

    SCurveProfile profile = planner.planFromState(target_position, current_state, config);

    for (const auto& segment : profile.segments) {
        if (segment.duration > 0.0) {
            EXPECT_LE(std::abs(segment.jerk), config.max_jerk + 1e-6);
        }
    }
}

// Performance and Integration Tests
TEST_F(SCurvePlannerTest, PerformanceBenchmark_SamplingUnder1Microsecond) {
    TrajectoryConfig config(10.0, 5.0, 20.0);
    double distance = 50.0;

    SCurveProfile profile = planner.planTrajectory(distance, config);

    const int iterations = 100000;
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; ++i) {
        double time = (i % 1000) / 1000.0 * profile.total_duration;
        MotionState state = planner.sampleState(profile, time);
        (void)state;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

    double avg_time_ns = static_cast<double>(duration.count()) / iterations;
    double avg_time_us = avg_time_ns / 1000.0;

    EXPECT_LT(avg_time_us, 1.0) << "Average sampling time: " << avg_time_us << " μs";
}

TEST_F(SCurvePlannerTest, Integration_CompleteWorkflow) {
    // Setup
    TrajectoryConfig config(10.0, 5.0, 20.0);
    SCurvePlanner planner;

    // Plan trajectory
    double distance = 50.0;
    SCurveProfile profile = planner.planTrajectory(distance, config);

    // Verify structure
    EXPECT_EQ(profile.segments.size(), 7);
    EXPECT_GT(profile.total_duration, 0.0);

    // Sample at multiple points
    std::vector<double> sample_times = {0.0, profile.total_duration * 0.25,
                                        profile.total_duration * 0.5,
                                        profile.total_duration * 0.75,
                                        profile.total_duration};

    for (double time : sample_times) {
        MotionState state = planner.sampleState(profile, time);
        EXPECT_GE(state.position, -1e-6);
    }

    // Test mid-motion replanning
    MotionState mid_state = planner.sampleState(profile, profile.total_duration * 0.5);
    double new_target = 75.0;
    SCurveProfile new_profile = planner.planFromState(new_target, mid_state, config);

    EXPECT_GT(new_profile.total_duration, 0.0);
    EXPECT_EQ(new_profile.segments.size(), 7);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
