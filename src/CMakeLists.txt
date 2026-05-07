add_library(rt_smoothing
    src/SCurvePlanner.cpp
)

target_include_directories(rt_smoothing PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(rt_smoothing PUBLIC cxx_std_17)
