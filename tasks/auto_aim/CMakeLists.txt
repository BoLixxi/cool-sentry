add_library(auto_aim STATIC
  armor.cpp classifier.cpp detector.cpp solver.cpp aimer.cpp
  target.cpp tracker.cpp voter.cpp shooter.cpp yolo.cpp
  yolos/yolov5.cpp yolos/yolov8.cpp yolos/yolo11.cpp
)
target_link_libraries(auto_aim PUBLIC io tools openvino::runtime)
