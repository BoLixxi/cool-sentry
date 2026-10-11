add_library(tools STATIC
  exiter.cpp extended_kalman_filter.cpp img_tools.cpp math_tools.cpp
  plotter.cpp trajectory.cpp recorder.cpp logger.cpp
)
target_link_libraries(tools PUBLIC ${OpenCV_LIBS} fmt::fmt spdlog::spdlog
  yaml-cpp nlohmann_json::nlohmann_json Eigen3::Eigen Threads::Threads)
