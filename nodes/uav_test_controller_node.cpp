#include <uav_ros_tests/uav_test_controller.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_integration_test");

  ros::NodeHandle nh;
  auto uav_test_controller_ptr = std::make_shared<uav_tests::UAVTestController>(nh);

  bool result = false;
  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.01).sleep();

    if (uav_test_controller_ptr->isFinished()) {
      result = uav_test_controller_ptr->getResult();
      break;
    }
  }

  return 0;
}