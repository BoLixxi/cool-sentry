#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  while (!exiter.exit()) {
    camera.read(img, t);
    static int frame_cnt = 0;
    if (frame_cnt++ % 100 == 0) {
        tools::logger()->info("Heartbeat: Camera got frame {}, processing...", frame_cnt);
    }
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);
    //cboard.bullet_speed = 28.0;
    auto command = aimer.aim(targets, t, cboard.bullet_speed);

// 1. 检查 YOLO 是否检测到装甲板
    if (!armors.empty()) {
        tools::logger()->info("--- Detected {} armors ---", armors.size());
    }

    // 2. 检查追踪器是否有目标 (targets 是一个 std::list)
    if (!targets.empty()) {
        // 打印当前追踪的目标数量
        tools::logger()->info("Target Locked! Tracking {} targets", targets.size());
        
        // 3. 打印发送给电控的具体指令数据
        // 根据 io/command.hpp，成员为 yaw, pitch, shoot
        tools::logger()->info("Send To MCU -> Yaw: {:.3f} | Pitch: {:.3f} | Shoot: {}", 
                              command.yaw, command.pitch, command.shoot);
    }

    cboard.send(command);

    cboard.send(command);

    
  }

  return 0;
}