#include "cboard.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"
#include "new_framework/modules/master_machine/vision_wire.h"

namespace io
{
CBoard::CBoard(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  const auto port = yaml["serial_port"] ? yaml["serial_port"].as<std::string>()
                                        : tools::read<std::string>(yaml, "can_interface");
  const auto baudrate = yaml["baudrate"].as<unsigned int>(115200);
  bullet_speed = yaml["bullet_speed"].as<double>(23.0);
  if (!std::isfinite(bullet_speed) || bullet_speed <= 0)
    throw std::runtime_error("bullet_speed must be positive (m/s)");
  timeout_ = std::chrono::milliseconds(yaml["serial_timeout_ms"].as<int>(1000));
  if (timeout_.count() <= 0) throw std::runtime_error("serial_timeout_ms must be positive");
  max_imu_age_ = std::chrono::milliseconds(yaml["max_imu_age_ms"].as<int>(100));
  if (max_imu_age_.count() <= 0) throw std::runtime_error("max_imu_age_ms must be positive");
  const auto protocol = yaml["serial_protocol"].as<std::string>("legacy19");
  seasky_ = protocol == "seasky_v1";
  if (!seasky_ && protocol != "legacy19")
    throw std::runtime_error("serial_protocol must be legacy19 or seasky_v1");
  receive_frame_size_ = seasky_ ? 19 : yaml["receive_frame_size"].as<size_t>(19);
  if (receive_frame_size_ != 18 && receive_frame_size_ != 19)
    throw std::runtime_error("receive_frame_size must be 18 or 19 for legacy protocol");

  serial_ = std::make_unique<serial::Serial>(
    port, baudrate, serial::Timeout::simpleTimeout(100));
  if (!serial_->isOpen()) throw std::runtime_error("Cannot open serial port: " + port);
  running_ = true;
  receive_thread_ = std::thread(&CBoard::receive_loop, this);

  std::unique_lock<std::mutex> lock(mutex_);
  if (!received_.wait_for(lock, timeout_, [this] {
        return samples_.size() >= 2 || !running_;
      }) || samples_.size() < 2) {
    lock.unlock();
    running_ = false;
    if (receive_thread_.joinable()) receive_thread_.join();
    throw std::runtime_error("CBoard IMU startup timeout: check USART wiring and frame format");
  }
  tools::logger()->info("[CBoard] USART ready on {}, bullet speed {} m/s", port, bullet_speed);
}

CBoard::~CBoard()
{
  running_ = false;
  received_.notify_all();
  if (receive_thread_.joinable()) receive_thread_.join();
  if (serial_ && serial_->isOpen()) serial_->close();
}

void CBoard::receive_loop()
{
  std::vector<uint8_t> buffer;
  VisionParser parser{};
  buffer.reserve(2048);
  try {
    while (running_) {
      std::array<uint8_t, 1024> incoming{};
      const auto count = std::min(serial_->available(), incoming.size());
      if (count == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      const auto n = serial_->read(incoming.data(), count);
      if (seasky_) {
        std::array<uint8_t, VISION_ATTITUDE_SIZE> frame{};
        for (size_t j = 0; j < n; ++j) {
          if (!VisionFeed(&parser, incoming[j], 2, frame.size(), frame.data())) continue;
          if (frame[6] != 0 || frame[7] != 0) continue;
          // Convert to the legacy payload layout below, reusing quaternion validation.
          buffer.push_back(0x55); buffer.push_back(0xAA);
          buffer.insert(buffer.end(), frame.begin() + 8, frame.begin() + 24);
          uint8_t checksum = 0;
          for (size_t k = buffer.size() - 18; k < buffer.size(); ++k) checksum += buffer[k];
          buffer.push_back(checksum);
        }
      } else {
        buffer.insert(buffer.end(), incoming.begin(), incoming.begin() + n);
      }
      const auto timestamp = std::chrono::steady_clock::now();

      size_t i = 0;
      while (i + receive_frame_size_ <= buffer.size()) {
        if (buffer[i] != 0x55 || buffer[i + 1] != 0xAA) {
          ++i;
          continue;
        }
        if (receive_frame_size_ == 19) {
          uint8_t checksum = 0;
          for (size_t j = 0; j < 18; ++j) checksum += buffer[i + j];
          if (checksum != buffer[i + 18]) {
            ++i;
            continue;
          }
        }
        std::array<float, 4> xyzw{};
        std::memcpy(xyzw.data(), buffer.data() + i + 2, 16);
        Eigen::Quaterniond q(xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
        const auto norm = q.norm();
        if (!q.coeffs().allFinite() || norm < 0.8 || norm > 1.2) {
          ++i;
          continue;
        }
        q.normalize();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          // Several frames in one read share a host timestamp. Keep the latest.
          if (!samples_.empty() && samples_.back().timestamp == timestamp)
            samples_.back().q = q;
          else
            samples_.push_back({q, timestamp});
          while (samples_.size() > 5000) samples_.pop_front();
        }
        received_.notify_all();
        i += receive_frame_size_;
      }
      buffer.erase(buffer.begin(), buffer.begin() + i);
    }
  } catch (const std::exception & e) {
    tools::logger()->error("[CBoard] USART receive failed: {}", e.what());
  }
  running_ = false;
  received_.notify_all();
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!received_.wait_for(lock, timeout_, [this, timestamp] {
        return !running_ || (!samples_.empty() && samples_.back().timestamp >= timestamp);
      })) {
    throw std::runtime_error("CBoard IMU timeout");
  }
  if (!running_) throw std::runtime_error("CBoard receive thread stopped");
  if (std::chrono::steady_clock::now() - samples_.back().timestamp > max_imu_age_)
    throw std::runtime_error("CBoard IMU data is stale");
  while (samples_.size() > 2 && samples_[1].timestamp <= timestamp)
    samples_.pop_front();
  if (timestamp <= samples_.front().timestamp) return samples_.front().q;
  const auto & a = samples_[0];
  const auto & b = samples_[1];
  const auto span = std::chrono::duration<double>(b.timestamp - a.timestamp).count();
  const auto offset = std::chrono::duration<double>(timestamp - a.timestamp).count();
  const auto k = std::clamp(offset / span, 0.0, 1.0);
  return a.q.slerp(k, b.q).normalized();
}

void CBoard::send(Command command) const
{
  if (!serial_ || !serial_->isOpen()) return;
  if (!std::isfinite(command.yaw) || !std::isfinite(command.pitch))
    command = {false, false, 0, 0};
  std::array<uint8_t, 16> data{0xAA, 0x55, 0x01};
  const int16_t control = command.control ? 1 : 0;
  const int16_t shoot = command.shoot ? 1 : 0;
  const float yaw = static_cast<float>(command.yaw);
  const float pitch = static_cast<float>(command.pitch);
  if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
    send({false, false, 0, 0});
    return;
  }
  if (seasky_) {
    std::array<uint8_t, VISION_COMMAND_SIZE> packet{};
    const float angles[2] = {yaw, pitch};
    const uint16_t flags = command.control ? (command.shoot ? 3 : 1) : 0;
    VisionPack(packet.data(), 1, flags, angles, 2);
    try {
      size_t sent = 0;
      while (sent < packet.size()) {
        const auto n = serial_->write(packet.data() + sent, packet.size() - sent);
        if (!n) throw std::runtime_error("USART write timeout");
        sent += n;
      }
    } catch (const std::exception & e) {
      tools::logger()->warn("[CBoard] USART send failed: {}", e.what());
    }
    return;
  }
  std::memcpy(data.data() + 3, &control, 2);
  std::memcpy(data.data() + 5, &shoot, 2);
  std::memcpy(data.data() + 7, &yaw, 4);
  std::memcpy(data.data() + 11, &pitch, 4);
  for (size_t i = 2; i < 15; ++i) data[15] += data[i];
  try {
    size_t sent = 0;
    while (sent < data.size()) {
      const auto n = serial_->write(data.data() + sent, data.size() - sent);
      if (n == 0) throw std::runtime_error("USART write timeout (incomplete command)");
      sent += n;
    }
  } catch (const std::exception & e) {
    tools::logger()->warn("[CBoard] USART send failed: {}", e.what());
  }
}
}  // namespace io
