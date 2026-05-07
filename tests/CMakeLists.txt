include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
)

if(NOT googletest_POPULATED)
    FetchContent_Populate(googletest)
    add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
endif()

enable_testing()

add_executable(test_scurve_planner test_scurve_planner.cpp)
target_link_libraries(test_scurve_planner
    gtest_main
    rt_smoothing
)

add_executable(scenario_runner scenario_runner.cpp)
target_link_libraries(scenario_runner rt_smoothing)

add_executable(arm_runner arm_runner.cpp)
target_link_libraries(arm_runner rt_smoothing)

add_executable(jerky_runner jerky_runner.cpp)
target_link_libraries(jerky_runner rt_smoothing)

add_executable(rt_arm_runner rt_arm_runner.cpp)
target_link_libraries(rt_arm_runner rt_smoothing)

# ── Bezier + Eigen/RL IK + S-Curve runner ───────────────────────────────────
find_package(Eigen3 REQUIRED)

# Robotics Library (RL) - 0.7.0
set(RL_INCLUDE_DIR "/usr/include/rl-0.7.0")
link_directories("/usr/lib/x86_64-linux-gnu")

add_executable(bezier_ik_runner bezier_ik_runner.cpp)
target_include_directories(bezier_ik_runner PRIVATE 
    ${EIGEN3_INCLUDE_DIR}
    ${RL_INCLUDE_DIR}
)
target_link_libraries(bezier_ik_runner 
    rt_smoothing
    rlkin rlmdl
)

include(GoogleTest)
gtest_discover_tests(test_scurve_planner)
