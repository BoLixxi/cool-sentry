# 代码结构

本工程当前只保留单海康相机的装甲板自瞄链路：

`src/standard.cpp` 是运行入口。每帧依次读取相机和 C 板四元数，执行 YOLO 装甲板识别、目标跟踪、坐标解算、弹道预测和射击判断，再通过 USART 向 C 板发送云台角度与开火标志。

- `io/camera.*`、`io/hikrobot/`：海康 USB 相机。
- `io/cboard.*`：C 板 USART。按 `55 AA + x/y/z/w 四个 float + 累加校验` 接收四元数，按 `AA 55 01 + control + shoot + yaw + pitch + 累加校验` 发送指令。
- `tasks/auto_aim/`：装甲板识别、分类、跟踪、坐标解算、瞄准和射击判断。
- `tools/`：自瞄依赖的数学、滤波、日志和图像工具。
- `configs/standard3.yaml`：实车配置。需要确认 `serial_port`、`bullet_speed`、相机参数和手眼标定参数。
- `calibration/`：相机和手眼标定程序。
- `tests/`：相机、C 板和离线视频测试程序。
- `io/ros2/`：原开源导航接口，默认不参与构建，可用 `-DBUILD_ROS2_NAV=ON` 单独启用。

构建：

```bash
cmake -S . -B build -DBUILD_DEBUG_TOOLS=ON
cmake --build build -j2
./build/standard configs/standard3.yaml
```

USART 帧格式已对照电控核实，本队使用 `receive_frame_size: 19` 并检查累加校验；18 字节仅兼容其他旧固件。电控需要修正二进制发送与指令解析，详见 `FIRMWARE_INTERFACE.md`。角度方向、遥控器启用条件和导航协议尚待实车对接。USART 自动测试为 `python3 tests/cboard_serial_test.py ./build_clean/cboard_serial_probe`。
