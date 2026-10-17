cmake_minimum_required(VERSION 3.16)
project(cool_sentry_workspace LANGUAGES CXX)

# The desktop auto-aim program is the only component built by this host CMake
# entry point.  The STM32 firmware in ele-con/ uses its own ARM toolchain
# and must be configured separately.
add_subdirectory(auto-aim)
