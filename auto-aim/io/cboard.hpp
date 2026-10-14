#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <serial/serial.h>
#include <string>
#include <thread>

#include "io/command.hpp"

namespace io
{
class CBoard
{
public:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  double bullet_speed;
  explicit CBoard(const std::string & config_path);
  ~CBoard();
  CBoard(const CBoard &) = delete;
  CBoard & operator=(const CBoard &) = delete;

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  void send(Command command) const;

private:
  std::unique_ptr<serial::Serial> serial_;
  std::thread receive_thread_;
  std::atomic<bool> running_{false};
  std::mutex mutex_;
  std::condition_variable received_;
  std::deque<IMUData> samples_;
  std::chrono::milliseconds timeout_{1000};
  std::chrono::milliseconds max_imu_age_{100};
  bool seasky_ = false;
  size_t receive_frame_size_ = 19;

  void receive_loop();
};
}  // namespace io
#endif
