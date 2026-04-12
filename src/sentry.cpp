#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/ros2/publish2nav.hpp"
#include "io/ros2/ros2.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  io::ROS2 ros2;
  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  //io::Camera back_camera("configs/camera.yaml");
  //io::USBCamera usbcam1("video0", config_path);
  //io::USBCamera usbcam2("video2", config_path);

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);

  cv::Mat img;

  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
   / Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
   //Eigen::Quaterniond q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    // recorder.record(img, q, timestamp);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo.detect(img);

    decider.get_invincible_armor(ros2.subscribe_enemy_status());

    decider.armor_filter(armors);

    // decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());

    decider.set_priority(armors);

    auto targets = tracker.track(armors, timestamp);

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
   // if (tracker.state() == "lost")
  //    command = decider.decide(yolo, gimbal_pos, usbcam1, usbcam2, back_camera);
  //  else
   //   command = aimer.aim(targets, timestamp, cboard.bullet_speed, cboard.shoot_mode);
    /// 核心状态机逻辑
    if (tracker.state() == "lost") {
      // 因为只有单相机，删掉了 decider.decide(...) 
      // 目标丢失时，停止发弹，云台保持原位（或者让电控接管去写巡逻逻辑）
      command.control = true;  // 视觉继续接管云台
      command.shoot = false;   // 绝对不许开火
      command.yaw = 0;         // 相对当前角度偏转为0
      command.pitch = 0;
    } else {
      // 如果追踪到目标，交给 aimer 预测并生成打击角度
      command = aimer.aim(targets, timestamp, cboard.bullet_speed, cboard.shoot_mode);
    }
    
    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
    /// --- 新增的打印测试逻辑 ---
    if (!targets.empty()) {
      // 如果锁定了目标，实时打印准备发送给电控的角度和射击指令
      tools::logger()->info("🔥 Sentry Locked! Targets: {} | Send -> Yaw: {:.2f}, Pitch: {:.2f}, Shoot: {}", 
                            targets.size(), command.yaw, command.pitch, command.shoot);
    } else {
      // 如果没有目标，加一个心跳包（每100帧打印一次，防止刷屏太快看不清）
      static int heartbeat = 0;
      if (heartbeat++ % 100 == 0) {
        tools::logger()->info("👀 Sentry searching... (Frame: {})", heartbeat);
      }
    }
    /// --------------------------

    cboard.send(command);

    /// ROS2通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    ros2.publish(target_info);
  }
  return 0;
}