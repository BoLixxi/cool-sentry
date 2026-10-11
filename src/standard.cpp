#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path | configs/standard3.yaml | YAML 配置文件路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  while (!exiter.exit()) {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    camera.read(img, timestamp);
    if (img.empty()) {
      cboard.send({false, false, 0, 0});
      continue;
    }

    try {
      const auto q = cboard.imu_at(timestamp - 1ms);
      solver.set_R_gimbal2world(q);
      auto armors = detector.detect(img);
      auto targets = tracker.track(armors, timestamp);
      auto command = aimer.aim(targets, timestamp, cboard.bullet_speed);
      const auto gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
      cboard.send(command);
    } catch (const std::exception & e) {
      cboard.send({false, false, 0, 0});
      tools::logger()->warn("Auto aim frame skipped: {}", e.what());
    }
  }

  cboard.send({false, false, 0, 0});
  return 0;
}
