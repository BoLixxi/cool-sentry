#include "io/camera.hpp"

#include <opencv2/opencv.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }"
  "{plot p         |                     | UDP 9870 输出相机指标到 PlotJuggler}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");
  auto plot = cli.has("plot");
  tools::Plotter plotter;
  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_stamp = std::chrono::steady_clock::now();
  bool first = true;
  size_t frames = 0;
  size_t long_intervals = 0;
  auto started = last_stamp;
  while (!exiter.exit()) {
    camera.read(img, timestamp);
    if (img.empty()) continue;
    ++frames;

    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;

    if (first) {
      started = timestamp;
      first = false;
      continue; // Exclude camera startup latency from frame timing.
    }
    if (dt <= 0) continue;
    if (dt > 0.025) ++long_intervals;
    const auto elapsed = tools::delta_time(timestamp, started);
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    const auto brightness = cv::mean(gray)[0];
    if (plot) plotter.plot({
      {"t", elapsed}, {"camera/frame_interval_ms", dt * 1000},
      {"camera/fps", 1 / dt}, {"camera/average_fps", (frames - 1) / elapsed},
      {"camera/brightness", brightness}, {"camera/frames", frames},
      {"camera/intervals_over_25ms", long_intervals}});
    if (frames % 100 == 0)
      tools::logger()->info("{:.2f} fps, {} frames, {} intervals >25ms", 1 / dt, frames, long_intervals);

    if (!display) continue;
    cv::putText(img, cv::format("%.1f fps | %.1f ms | >25ms: %zu", 1 / dt, dt * 1000, long_intervals),
                {20, 35}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
    cv::imshow("Hikrobot live - q to quit", img);
    if (cv::waitKey(1) == 'q') break;
  }
}
