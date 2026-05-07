#include "rt_smoothing/SCurvePlanner.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

using namespace rt_smoothing;

int main(int argc, char* argv[]) {
    std::vector<double> target_positions = { 1.5, -1.0, 1.2, 0.5, -0.8, 2.0 };
    
    if (argc == 7) {
        for (int i = 0; i < 6; ++i) {
            target_positions[i] = std::stod(argv[i + 1]);
        }
    } else if (argc > 1) {
        std::cerr << "Usage: " << argv[0] << " [j0 j1 j2 j3 j4 j5]" << std::endl;
        std::cerr << "Using default positions." << std::endl;
    }

    // Lowering limits for the payload stability test so we don't break static friction
    std::vector<TrajectoryConfig> configs = {
        { 1.0, 0.5, 2.0 }, // Joint 1
        { 1.0, 0.5, 2.0 }, // Joint 2
        { 1.0, 0.5, 2.0 }, // Joint 3
        { 1.0, 0.5, 2.0 }, // Joint 4
        { 1.0, 0.5, 2.0 }, // Joint 5
        { 1.0, 0.5, 2.0 }  // Joint 6
    };

    SCurvePlanner planner;
    std::vector<SCurveProfile> profiles = planner.planSynchronizedTrajectory(target_positions, configs);

    if (profiles.empty()) {
        std::cerr << "Failed to generate synchronized profiles" << std::endl;
        return 1;
    }

    double max_duration = profiles[0].total_duration;
    std::cout << "Synchronized Duration: " << max_duration << " seconds" << std::endl;

    std::ofstream out("joint_trajectory.csv");
    out << "time,j0,j1,j2,j3,j4,j5\n";

    for (double t = 0.0; t <= max_duration + 0.001; t += 0.01) {
        out << std::fixed << std::setprecision(4) << t;
        for (const auto& profile : profiles) {
            MotionState state = planner.sampleState(profile, t);
            out << "," << state.position;
        }
        out << "\n";
    }

    return 0;
}
