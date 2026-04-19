#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/ros2/publish2nav.hpp"// 往 ROS2 导航系统发数据的接口
#include "io/ros2/ros2.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"// 射击决策逻辑
#include "tasks/auto_aim/solver.hpp"// 3D坐标解算逻辑 (PnP)
#include "tasks/auto_aim/tracker.hpp"// 目标追踪逻辑 (卡尔曼滤波)
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"// 优雅退出程序的工具 (比如处理 Ctrl+C)
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"// 数学工具（欧拉角、四元数等）
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;
// 定义命令行解析器的规则：
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // 实例化一些基础工具对象
  tools::Exiter exiter;// 监听退出信号
  tools::Plotter plotter;// 绘图器准备
  tools::Recorder recorder;// 录像机准备
  // 初始化 OpenCV 的命令行解析器
  cv::CommandLineParser cli(argc, argv, keys);
  // 如果你在终端运行程序时加了 -h，就会打印帮助并退出
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  // 获取配置文件的路径，保存到 config_path 字符串变量中
  auto config_path = cli.get<std::string>(0);
  // --- 模块初始化阶段 (搭积木) ---
  io::ROS2 ros2;// 启动 ROS2 节点
  io::CBoard cboard(config_path);// 启动与单片机的串口通信
  io::Camera camera(config_path);// 初始化主相机

  //io::Camera back_camera("configs/camera.yaml");
  //io::USBCamera usbcam1("video0", config_path);
  //io::USBCamera usbcam2("video2", config_path);

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);// 解算器
  auto_aim::Tracker tracker(config_path, solver);// 追踪器，注意追踪器依赖解算器
  auto_aim::Aimer aimer(config_path);// 瞄准器
  auto_aim::Shooter shooter(config_path);
  // 初始化哨兵机器人的全向决策调度器，它会根据当前的视觉输入和追踪状态，决定打哪个目标，怎么打，以及是否切换到全向感知模式
  omniperception::Decider decider(config_path);
  //Input: 相机类产生一帧 cv::Mat 图像，经过 YOLO 检测出装甲板
  //Tracker 追踪装甲板并记住目标
  //Aimer 根据目标的运动轨迹预测未来位置并生成云台控制指令
  //Shooter 根据指令和目标信息决定是否发射，CBoard 接收指令控制云台和发弹，ROS2 发布目标信息供其他模块使用。
  cv::Mat img;// 定义一个矩阵变量 img，用来存放相机的画面
  // 定义一个时间点变量，用来记录每一帧图像生成的时间
  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  // --- 主循环 (生命线) ---
  // 只要没收到退出信号(exiter.exit()为假)，就一直死循环运行
  while (!exiter.exit()) {
    camera.read(img, timestamp);// 1. 【感知】从相机读取一帧图像存入 img，并记录这一刻的时间戳
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
   //Eigen::Quaterniond q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
   // recorder.record(img, q, timestamp);

    /// 自瞄核心逻辑
    /// --- 自瞄核心逻辑 ---
    // 2. 将当前云台的姿态告诉解算器，以便建立世界坐标系
    solver.set_R_gimbal2world(q);
    // 3. 利用数学工具，把刚才的四元数转换为欧拉角(Yaw, Pitch, Roll) 作为当前云台的实际位置
    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    // 4. 【检测】将图像送入 YOLO 网络，找出图片中所有的装甲板 (armors)
    auto armors = yolo.detect(img);
    // 5. 【决策过滤】从 ROS2 接收敌方状态，判断哪些装甲板处于“无敌状态”
    decider.get_invincible_armor(ros2.subscribe_enemy_status());
    // 剔除掉无敌的、或者不该打的装甲板（比如队友的），并且给剩下的装甲板按照预设的优先级排序
    decider.armor_filter(armors);
    // 通过 ROS2 获取外部指定的自瞄目标 
    // decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());
    // 6. 为剩下的合法装甲板设置优先级
    decider.set_priority(armors);
    // 7. 【追踪】将当前看到的装甲板送入追踪器，进行滤波和历史匹配，输出持续的追踪目标 (targets)
    auto targets = tracker.track(armors, timestamp);
    // 定义一个默认的指令包：不接管控制(false), 不开火(false), Yaw=0, Pitch=0
    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
   // if (tracker.state() == "lost")
  //    command = decider.decide(yolo, gimbal_pos, usbcam1, usbcam2, back_camera);
  //  else
   //   command = aimer.aim(targets, timestamp, cboard.bullet_speed, cboard.shoot_mode);
    /// 核心状态机逻辑
    // 8. 根据追踪器的状态决定动作
    if (tracker.state() == "lost") {// 状态：目标丢失
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
    // 10. 【开火判定】判断枪口是否已经对准了预测位置，如果对准了，把 command.shoot 设为 true (允许开火)
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
    // 11. 【通讯】把最终决定的指令 (Yaw, Pitch, Shoot) 通过串口发送给单片机电控板
    cboard.send(command);

    /// ROS2通信
    // 12. 整理当前的目标信息（坐标、距离等）
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);
    // 把这些信息发布到 ROS2 网络，供导航模块或其他模块使用
    ros2.publish(target_info);
  }
  return 0;
}