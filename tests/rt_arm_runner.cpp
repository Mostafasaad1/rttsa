#include "rt_smoothing/SCurvePlanner.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <sstream>

using namespace rt_smoothing;

// Helper to make socket non-blocking
void setNonBlocking(int sock) {
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
}

int main() {
    int send_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in send_addr;
    memset(&send_addr, 0, sizeof(send_addr));
    send_addr.sin_family = AF_INET;
    send_addr.sin_port = htons(5000);
    inet_pton(AF_INET, "127.0.0.1", &send_addr.sin_addr);

    int recv_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in recv_addr;
    memset(&recv_addr, 0, sizeof(recv_addr));
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(5001);
    recv_addr.sin_addr.s_addr = INADDR_ANY;
    bind(recv_sock, (struct sockaddr*)&recv_addr, sizeof(recv_addr));
    setNonBlocking(recv_sock);

    // V=1.0, A=0.5, J=2.0 for physical stability
    std::vector<TrajectoryConfig> configs(6, {1.0, 0.5, 2.0}); 
    std::vector<MotionState> current_states(6, {0.0, 0.0, 0.0});
    std::vector<double> current_targets(6, 0.0);
    
    SCurvePlanner planner;
    std::vector<SCurveProfile> profiles(6);
    std::vector<double> current_times(6, 0.0);

    // Initial plan (stays at 0)
    for (int i = 0; i < 6; ++i) {
        profiles[i] = planner.planFromState(0.0, current_states[i], configs[i]);
    }

    std::cout << "Real-Time UDP Trajectory Server Running..." << std::endl;
    std::cout << "Listening for targets on port 5001. Sending states to port 5000." << std::endl;

    auto next_tick = std::chrono::steady_clock::now();
    const auto tick_duration = std::chrono::milliseconds(10); // 100Hz

    char buffer[1024];

    while (true) {
        // 1. Check for new targets
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int bytes = recvfrom(recv_sock, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&client_addr, &client_len);
        
        if (bytes > 0) {
            buffer[bytes] = '\0';
            std::string msg(buffer);
            std::stringstream ss(msg);
            std::string item;
            std::vector<double> new_targets;
            while (std::getline(ss, item, ',')) {
                try { new_targets.push_back(std::stod(item)); } catch(...) {}
            }
            
            if (new_targets.size() == 6) {
                std::cout << "New targets received: " << msg << std::endl;
                for (int i = 0; i < 6; ++i) {
                    if (std::abs(new_targets[i] - current_targets[i]) > 0.001) {
                        current_targets[i] = new_targets[i];
                        // Mid-motion replanning from current state!
                        profiles[i] = planner.planFromState(new_targets[i], current_states[i], configs[i]);
                        current_times[i] = 0.0;
                    }
                }
            }
        }

        // 2. Advance state and sample
        std::stringstream out_msg;
        for (int i = 0; i < 6; ++i) {
            if (current_times[i] <= profiles[i].total_duration) {
                current_states[i] = planner.sampleState(profiles[i], current_times[i]);
                current_times[i] += 0.01; // 10ms
            } else {
                current_states[i] = planner.sampleState(profiles[i], profiles[i].total_duration);
            }
            out_msg << current_states[i].position << (i == 5 ? "" : ",");
        }

        // 3. Send state
        std::string payload = out_msg.str();
        sendto(send_sock, payload.c_str(), payload.length(), 0, (struct sockaddr*)&send_addr, sizeof(send_addr));

        // 4. Sleep until next 10ms tick
        next_tick += tick_duration;
        std::this_thread::sleep_until(next_tick);
    }

    close(send_sock);
    close(recv_sock);
    return 0;
}
