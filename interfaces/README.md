# Shared Interfaces

这里存放导航、自瞄和电控之间的共享通信契约。功能模块可以依赖本目录，但本目录不依赖任何功能模块。

- `vision_wire.h`：SeaSky v1 视觉 USART 帧、CRC、解析器及命令结构。由 `auto-aim/io/cboard.cpp` 和 `ele-con/` 共同使用。

修改协议时应在同一个提交中同步修改发送端、接收端、配置、文档和主机协议测试，避免两端版本漂移。
