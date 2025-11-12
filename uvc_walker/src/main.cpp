// #include <rclcpp/rclcpp.hpp>
// #include "uvc_walker/uvc_walker.hpp"

// int main(int argc, char * argv[])
// {
//   // ROS2 초기화
//   rclcpp::init(argc, argv);

//   // 노드 생성 및 실행
//   auto node = std::make_shared<UVCWalker>();
//   rclcpp::spin(node);

//   // 종료
//   rclcpp::shutdown();
//   return 0;
// }



#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "uvc_walker/uvc_walker.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UVCWalker>();

  // 멀티스레드 executor: IMU 콜백과 타이머를 병렬로 실행
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
