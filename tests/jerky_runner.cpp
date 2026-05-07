#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>

int main(int argc, char* argv[]) {
    // Same default targets as arm_runner
    std::vector<double> target_positions = { 1.5, -1.0, 1.2, 0.5, -0.8, 2.0 };
    std::vector<double> start_positions = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    
    if (argc == 7) {
        for (int i = 0; i < 6; ++i) {
            target_positions[i] = std::stod(argv[i + 1]);
        }
    }

    // Fixed short duration to force high speed and infinite jerk (step-velocity)
    double duration = 1.0; 

    std::ofstream out("joint_trajectory.csv");
    out << "time,j0,j1,j2,j3,j4,j5\n";

    // Generate linear interpolation (constant velocity, infinite acceleration at start/stop)
    for (double t = 0.0; t <= duration + 0.001; t += 0.01) {
        out << std::fixed << std::setprecision(4) << t;
        double ratio = t / duration;
        if (ratio > 1.0) ratio = 1.0;

        for (int i = 0; i < 6; ++i) {
            double pos = start_positions[i] + (target_positions[i] - start_positions[i]) * ratio;
            out << "," << pos;
        }
        out << "\n";
    }

    std::cout << "Jerky Linear Trajectory Generated. Duration: " << duration << " seconds" << std::endl;
    return 0;
}
