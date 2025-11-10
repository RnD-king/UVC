#include <rclcpp/rclcpp.hpp>
#include "uvc_walker/uvc_walker.hpp"

int main(int argc, char * argv[])
{
  // ROS2 초기화
  rclcpp::init(argc, argv);

  // 노드 생성 및 실행
  auto node = std::make_shared<UVCWalker>();
  rclcpp::spin(node);

  // 종료
  rclcpp::shutdown();
  return 0;
}
