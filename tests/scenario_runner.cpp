#include "rt_smoothing/SCurvePlanner.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace rt_smoothing;

int main(int argc, char** argv) {
    // Default values
    double max_v = 10.0;
    double max_a = 5.0;
    double max_j = 20.0;
    double start_p = 0.0;
    double start_v = 5.0;
    double start_a = 0.0;
    double target_p = 50.0;

    if (argc >= 8) {
        max_v = std::stod(argv[1]);
        max_a = std::stod(argv[2]);
        max_j = std::stod(argv[3]);
        start_p = std::stod(argv[4]);
        start_v = std::stod(argv[5]);
        start_a = std::stod(argv[6]);
        target_p = std::stod(argv[7]);
    }
    
    TrajectoryConfig config{ max_v, max_a, max_j };
    MotionState start{ start_p, start_v, start_a };
    
    SCurvePlanner planner;
    SCurveProfile profile = planner.planFromState(target_p, start, config);
    
    std::ofstream out("trajectory.csv");
    out << "time,position,velocity,acceleration,jerk\n";
    
    double accumulated_time = 0.0;
    for (double t = 0.0; t <= profile.total_duration + 0.001; t += 0.01) {
        MotionState current = planner.sampleState(profile, t);
        
        // Find jerk for current time
        double current_jerk = 0.0;
        double seg_time = 0.0;
        for (const auto& seg : profile.segments) {
            if (t <= seg_time + seg.duration + 1e-6) {
                current_jerk = seg.jerk;
                break;
            }
            seg_time += seg.duration;
        }

        out << std::fixed << std::setprecision(4) 
            << t << "," << current.position << "," 
            << current.velocity << "," << current.acceleration << ","
            << current_jerk << "\n";
    }
    
    return 0;
}
