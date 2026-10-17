#include <cmath>
#include <iostream>
#include <thread>

#include "io/cboard.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) return 1;
  try {
    io::CBoard board(argv[1]);
    const auto q = board.imu_at(std::chrono::steady_clock::now());
    if (std::abs(q.w() - 1) > 1e-6) return 2;
    board.send({true, false, 0.25, -0.5});
    std::cout << "IMU_OK" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    try {
      board.imu_at(std::chrono::steady_clock::now() - std::chrono::seconds(2));
      return 3;
    } catch (const std::runtime_error & e) {
      if (std::string(e.what()) != "CBoard IMU data is stale") throw;
      std::cout << "STALE_OK" << std::endl;
    }
    try {
      board.imu_at(std::chrono::steady_clock::now());
      return 4;
    } catch (const std::runtime_error & e) {
      if (std::string(e.what()) != "CBoard IMU timeout") throw;
      std::cout << "TIMEOUT_OK" << std::endl;
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 5;
  }
}
