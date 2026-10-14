# Cool Sentry

哨兵机器人导航、自瞄与电控联调仓库。三个主要功能目录彼此独立，通信协议集中在 `interfaces/`，联调和历史资料集中在 `docs/`。

## 目录

- `auto-aim/`：海康相机、装甲板识别、目标跟踪、弹道解算和视觉串口通信。
- `navigation/`：ROS2 导航桥接、MID360/Point-LIO 配置及验收脚本。
- `ele-con/`：RoboMaster C 型开发板 STM32 电控工程。
- `interfaces/`：上位机与电控共同使用的通信协议定义。
- `docs/`：部署交接、旧接口和联调资料。

## 自瞄构建

自瞄仍可独立构建，运行时以 `auto-aim/` 为工作目录，使 YAML 中的模型和资源相对路径保持有效：

```bash
cd auto-aim
cmake -S . -B build_clean -DBUILD_DEBUG_TOOLS=ON
cmake --build build_clean -j2
./build_clean/standard configs/sentry_step1.yaml
```

也可以从仓库根目录统一构建自瞄：

```bash
cmake -S . -B build
cmake --build build -j2
cd auto-aim
../build/auto-aim/standard configs/sentry_step1.yaml
```

## 电控构建

`ele-con/` 使用独立 ARM 工具链，不能与主机自瞄程序共用同一次 CMake 配置：

```bash
cmake -S ele-con -B ele-con/build
cmake --build ele-con/build
```

当前仓库已完成导航 USB 与视觉 USART 双链路通信验收，但默认仍为裸板通信模式；启用底盘、云台和发射机构前请先阅读 `navigation/README.md` 和 `ele-con/VISION_PROTOCOL.md`。
