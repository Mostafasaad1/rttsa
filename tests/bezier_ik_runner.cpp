/**
 * bezier_ik_runner.cpp
 *
 * Cartesian Bezier Path Planner with Numerical IK & S-Curve Smoothing
 *
 * Pipeline:
 *   1. Two 3D Cartesian waypoints (A, B) received via argv
 *   2. Cubic Bezier curve generated in Cartesian space
 *   3. Numerical Inverse Jacobian IK (Damped Least Squares) at each waypoint
 *      - UR5 DH parameters hard-coded (exact same as CoppeliaSim model)
 *      - End-effector orientation locked UPRIGHT (tray stays flat)
 *   4. Raw IK joints smoothed by our S-Curve SCurvePlanner
 *   5. States streamed via UDP port 5000 → rt_sim_runner.py → CoppeliaSim
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <sstream>
#include <thread>
#include <chrono>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <Eigen/Dense>

#include "rt_smoothing/SCurvePlanner.hpp"

using namespace rt_smoothing;

// ── UR5 DH Parameters ─────────────────────────────────────────────────────
// Standard UR5 Modified DH: [d, a, alpha, theta_offset]
struct DHParam { double d, a, alpha, theta; };
static const DHParam UR5_DH[6] = {
    { 0.0892,  0.0,    M_PI_2,  0.0 },
    { 0.0,    -0.425,  0.0,     0.0 },
    { 0.0,    -0.3922, 0.0,     0.0 },
    { 0.1093,  0.0,    M_PI_2,  0.0 },
    { 0.09475, 0.0,   -M_PI_2,  0.0 },
    { 0.0823,  0.0,    0.0,     0.0 }
};

// ── DH Transform ──────────────────────────────────────────────────────────
Eigen::Matrix4d dhTransform(const DHParam& dh, double q) {
    double th = q + dh.theta;
    double ct = cos(th), st = sin(th);
    double ca = cos(dh.alpha), sa = sin(dh.alpha);

    Eigen::Matrix4d T;
    T << ct, -st*ca,  st*sa,  dh.a*ct,
         st,  ct*ca, -ct*sa,  dh.a*st,
         0,   sa,     ca,     dh.d,
         0,   0,      0,      1;
    return T;
}

// ── Forward Kinematics ─────────────────────────────────────────────────────
Eigen::Matrix4d forwardKinematics(const std::vector<double>& q) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 6; ++i)
        T = T * dhTransform(UR5_DH[i], q[i]);
    return T;
}

// ── Geometric Jacobian (6x6) ───────────────────────────────────────────────
Eigen::MatrixXd computeJacobian(const std::vector<double>& q) {
    Eigen::MatrixXd J(6, 6);

    // Compute all frame transforms
    std::vector<Eigen::Matrix4d> T(7);
    T[0] = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 6; ++i)
        T[i+1] = T[i] * dhTransform(UR5_DH[i], q[i]);

    Eigen::Vector3d pe = T[6].block<3,1>(0,3); // end-effector position

    for (int i = 0; i < 6; ++i) {
        Eigen::Vector3d zi = T[i].block<3,1>(0,2); // z-axis of frame i
        Eigen::Vector3d pi = T[i].block<3,1>(0,3); // origin of frame i

        J.block<3,1>(0,i) = zi.cross(pe - pi);  // linear part
        J.block<3,1>(3,i) = zi;                  // angular part
    }
    return J;
}

// ── Damped Least Squares IK ────────────────────────────────────────────────
// Solves for q such that FK(q) → target pose
// Keeps end-effector orientation UPRIGHT (tray flat)
bool inverseKinematics(const Eigen::Vector3d& targetPos,
                       std::vector<double>& q,
                       int maxIter = 150,
                       double poseTol = 5e-4)
{
    // Desired end-effector rotation: tray faces UP (Z-axis of EE points down into world)
    // This represents the UR5 tool0 pointing downward (common pick-and-place orientation)
    Eigen::Matrix3d R_des;
    R_des <<  1,  0,  0,
              0, -1,  0,
              0,  0, -1;

    double lambda = 0.01; // damping factor

    for (int iter = 0; iter < maxIter; ++iter) {
        Eigen::Matrix4d T = forwardKinematics(q);
        Eigen::Vector3d pos_err = targetPos - T.block<3,1>(0,3);

        // Orientation error using rotation matrix log
        Eigen::Matrix3d R_cur = T.block<3,3>(0,0);
        Eigen::Matrix3d R_err = R_des * R_cur.transpose();
        // Convert to axis-angle error vector
        double angle = std::acos(std::max(-1.0, std::min(1.0,
                        (R_err.trace() - 1.0) / 2.0)));
        Eigen::Vector3d ori_err = Eigen::Vector3d::Zero();
        if (std::abs(angle) > 1e-6) {
            ori_err << R_err(2,1)-R_err(1,2),
                       R_err(0,2)-R_err(2,0),
                       R_err(1,0)-R_err(0,1);
            ori_err *= angle / (2.0 * std::sin(angle));
        }

        Eigen::VectorXd err(6);
        err << pos_err, ori_err;

        double errNorm = err.norm();
        if (errNorm < poseTol) return true;

        Eigen::MatrixXd J = computeJacobian(q);
        // Damped least-squares: dq = J^T (J J^T + λ²I)^{-1} err
        Eigen::MatrixXd JJt = J * J.transpose();
        Eigen::VectorXd dq = J.transpose() *
            (JJt + lambda * lambda * Eigen::MatrixXd::Identity(6,6)).inverse() * err;

        // Clamp step size for stability
        double stepScale = std::min(1.0, 0.2 / (dq.norm() + 1e-9));
        for (int i = 0; i < 6; ++i)
            q[i] += stepScale * dq(i);
    }
    return false;
}

// ── Bezier Math ─────────────────────────────────────────────────────────────
struct Vec3 { double x, y, z; };

Eigen::Vector3d cubicBezier(const Vec3& p0, const Vec3& p1,
                             const Vec3& p2, const Vec3& p3, double t) {
    double mt = 1.0 - t;
    double b0=mt*mt*mt, b1=3*mt*mt*t, b2=3*mt*t*t, b3=t*t*t;
    return { b0*p0.x+b1*p1.x+b2*p2.x+b3*p3.x,
             b0*p0.y+b1*p1.y+b2*p2.y+b3*p3.y,
             b0*p0.z+b1*p1.z+b2*p2.z+b3*p3.z };
}

// ── Main ───────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    // Expecting triplets of coordinates (x, y, z) + 1 arc height
    if (argc < 4 || (argc - 2) % 3 != 0) {
        std::cerr << "Usage: " << argv[0] << " x1 y1 z1 [x2 y2 z2 ...] arc_height\n";
        return 1;
    }

    std::vector<Eigen::Vector3d> waypoints_cart;
    for (int i = 1; i < argc - 1; i += 3) {
        waypoints_cart.push_back({ std::stod(argv[i]), std::stod(argv[i+1]), std::stod(argv[i+2]) });
    }
    double arcH = std::stod(argv[argc - 1]);

    const int N_per_segment = 15;
    std::vector<double> q = { 0.0, -M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, 0.0 };
    std::vector<std::vector<double>> waypoints_joint;

    std::vector<Eigen::Vector3d> wps;
    for (size_t i = 0; i < waypoints_cart.size(); ++i) {
        wps.push_back(waypoints_cart[i]);
        if (i < waypoints_cart.size() - 1 && arcH > 0.0) {
            Eigen::Vector3d mid = (waypoints_cart[i] + waypoints_cart[i+1]) / 2.0;
            mid.z() += arcH;
            wps.push_back(mid);
        }
    }

    std::vector<Eigen::Vector3d> T(wps.size());
    for (size_t i = 0; i < wps.size(); ++i) {
        if (i == 0) T[i] = wps[1] - wps[0];
        else if (i == wps.size() - 1) T[i] = wps.back() - wps[wps.size() - 2];
        else T[i] = 0.5 * (wps[i+1] - wps[i-1]);
    }

    std::cout << "Solving IK for " << waypoints_cart.size() << " user waypoints (" 
              << wps.size() << " total including arc points, "
              << (wps.size() - 1) * N_per_segment << " IK steps)...\n";

    for (size_t i = 0; i < wps.size() - 1; ++i) {
        Eigen::Vector3d p0 = wps[i];
        Eigen::Vector3d p1 = wps[i] + T[i] / 3.0;
        Eigen::Vector3d p2 = wps[i+1] - T[i+1] / 3.0;
        Eigen::Vector3d p3 = wps[i+1];

        for (int j = 0; j < N_per_segment; ++j) {
            double t = static_cast<double>(j) / N_per_segment;
            double mt = 1.0 - t;
            Eigen::Vector3d pt = mt*mt*mt*p0 + 3*mt*mt*t*p1 + 3*mt*t*t*p2 + t*t*t*p3;

            std::vector<double> qSol = q;
            if (inverseKinematics(pt, qSol)) {
                q = qSol;
            } else {
                std::cerr << "  IK warning at segment " << i << " t=" << t << ", using last good.\n";
            }
            waypoints_joint.push_back(q);
        }
    }
    // Add the final point
    std::vector<double> qFinal = q;
    inverseKinematics(waypoints_cart.back(), qFinal);
    waypoints_joint.push_back(qFinal);

    // ── UDP socket ───────────────────────────────────────────────────────
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5000);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    std::cout << "Streaming Multi-Point Path to CoppeliaSim...\n";

    TrajectoryConfig cfg{ 2.0, 1.5, 8.0 }; // Slightly higher limits for multi-point
    SCurvePlanner planner;
    const double DT = 0.02;
    std::vector<double> lastQ = waypoints_joint[0];

    for (size_t wi = 1; wi < waypoints_joint.size(); ++wi) {
        const auto& tgt = waypoints_joint[wi];
        std::vector<double> dists(6);
        std::vector<TrajectoryConfig> cfgs(6, cfg);
        for (int j = 0; j < 6; ++j) dists[j] = tgt[j] - lastQ[j];

        auto profiles = planner.planSynchronizedTrajectory(dists, cfgs);
        double dur = profiles[0].total_duration;

        for (double t = 0.0; t <= dur; t += DT) {
            std::stringstream ss;
            for (int j = 0; j < 6; ++j) {
                auto st = planner.sampleState(profiles[j], t);
                ss << (lastQ[j] + st.position) << (j<5 ? "," : "");
            }
            std::string p = ss.str();
            sendto(sock, p.c_str(), p.size(), 0, (struct sockaddr*)&addr, sizeof(addr));
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
        lastQ = tgt;
    }

    std::cout << "Multi-point trajectory complete.\n";
    close(sock);
    return 0;
}
