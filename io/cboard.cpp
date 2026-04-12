#include "cboard.hpp"

#include <thread>
#include <atomic>
#include <serial/serial.h>

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{

// 全局串口变量
static serial::Serial* g_serial = nullptr;
static std::thread g_recv_thread;
static std::atomic<bool> g_running{false};

// 接收线程函数
// 接收线程函数
static void receiveLoop(tools::ThreadSafeQueue<CBoard::IMUData>* queue)
{
    uint8_t buffer[1024];
    size_t buffer_len = 0; // 静态缓存区，用于拼接碎片数据
    
    while (g_running && g_serial && g_serial->isOpen()) {
        size_t available = g_serial->available();
        if (available > 0) {
            // 防止溢出保护
            if (buffer_len + available > 1024) {
                buffer_len = 0; // 如果数据积压严重，清空重来
            }
            
            // 将新读到的碎片数据拼接到 buffer 后面
            size_t n = g_serial->read(buffer + buffer_len, available);
            buffer_len += n;
            auto timestamp = std::chrono::steady_clock::now();
            
            size_t i = 0;
            // 你的完整一帧数据是 19 字节 (55 aa + 16字节四元数 + 1字节尾)
            while (i + 19 <= buffer_len) {
                if (buffer[i] == 0x55 && buffer[i+1] == 0xAA) {
                    // 找到帧头，提取 16 字节的 payload
                    const uint8_t* payload = buffer + i + 2;
                    float x = *(float*)(payload);
                    float y = *(float*)(payload + 4);
                    float z = *(float*)(payload + 8);
                    float w = *(float*)(payload + 12);
                    
                    Eigen::Quaterniond q(w, x, y, z);
                    q.normalize();
                    
                    queue->push({q, timestamp}); // 推送给主线程
                    
                    i += 19; // 成功解析一帧，指针向后跳 19 个字节
                } else {
                    i++; // 不是帧头，向后移 1 字节继续寻找
                }
            }
            
            // 将没有凑够一帧的残缺数据移到 buffer 头部，等待下次循环拼接
            if (i < buffer_len) {
                std::memmove(buffer, buffer + i, buffer_len - i);
                buffer_len -= i;
            } else {
                buffer_len = 0; // 正好处理完
            }
        } else {
            // 没有数据时稍微休眠，防止CPU占用100%
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

CBoard::CBoard(const std::string & config_path)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(0),
  queue_(5000)
{
    auto yaml = tools::load(config_path);
    
    quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
    bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
    send_canid_ = tools::read<int>(yaml, "send_canid");
    
    std::string serial_port;
    int baudrate = 115200;
    
    if (!yaml["can_interface"]) {
        throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
    }
    serial_port = yaml["can_interface"].as<std::string>();
    if (yaml["baudrate"]) {
        baudrate = yaml["baudrate"].as<int>();
    }
    
    // 初始化串口
    g_serial = new serial::Serial(serial_port, baudrate);
    g_serial->setBytesize(serial::eightbits);
    g_serial->setParity(serial::parity_none);
    g_serial->setStopbits(serial::stopbits_one);
    g_serial->setFlowcontrol(serial::flowcontrol_none);
    
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    g_serial->setTimeout(timeout);
    
    if (!g_serial->isOpen()) {
        try {
            g_serial->open();
        } catch (const std::exception& e) {
            tools::logger()->error("Failed to open serial port: {}", e.what());
        }
    }
    
    if (g_serial->isOpen()) {
        tools::logger()->info("[Cboard] Serial port opened on {}", serial_port);
        g_running = true;
        g_recv_thread = std::thread(receiveLoop, &queue_);
    } else {
        tools::logger()->error("[Cboard] Failed to open serial port!");
    }
    
    tools::logger()->info("[Cboard] Waiting for data...");
    queue_.pop(data_ahead_);
    queue_.pop(data_behind_);
    tools::logger()->info("[Cboard] Opened.");
}

CBoard::~CBoard()
{
    g_running = false;
    if (g_recv_thread.joinable()) {
        g_recv_thread.join();
    }
    if (g_serial) {
        if (g_serial->isOpen()) g_serial->close();
        delete g_serial;
        g_serial = nullptr;
    }
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
    if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

    while (true) {
        queue_.pop(data_behind_);
        if (data_behind_.timestamp > timestamp) break;
        data_ahead_ = data_behind_;
    }

    Eigen::Quaterniond q_a = data_ahead_.q.normalized();
    Eigen::Quaterniond q_b = data_behind_.q.normalized();
    auto t_a = data_ahead_.timestamp;
    auto t_b = data_behind_.timestamp;
    auto t_c = timestamp;
    std::chrono::duration<double> t_ab = t_b - t_a;
    std::chrono::duration<double> t_ac = t_c - t_a;

    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

    return q_c;
}

void CBoard::send(Command command) const
{
    if (!g_serial || !g_serial->isOpen()) return;
    
    uint8_t data[20];
    int idx = 0;
    
    data[idx++] = 0xAA;
    data[idx++] = 0x55;
    data[idx++] = 0x01;
    
    *(int16_t*)(data + idx) = (int16_t)(command.control ? 1 : 0);
    idx += 2;
    
    *(int16_t*)(data + idx) = (int16_t)(command.shoot ? 1 : 0);
    idx += 2;
    
    *(float*)(data + idx) = (float)command.yaw;
    idx += 4;
    
    *(float*)(data + idx) = (float)command.pitch;
    idx += 4;
    
    uint8_t checksum = 0;
    for (int i = 2; i < idx; i++) {
        checksum += data[i];
    }
    data[idx++] = checksum;
    
    try {
        g_serial->write(data, idx);
    } catch (const std::exception & e) {
        tools::logger()->warn("Send failed: {}", e.what());
    }
}

std::string CBoard::read_yaml(const std::string & config_path)
{
    return "";
}

}  // namespace io