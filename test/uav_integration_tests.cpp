#include <gtest/gtest.h>
#include <ros/ros.h>

#include <uav_ros_tests/uav_test_controller.hpp>

std::shared_ptr<uav_tests::UAVTestController> uav_test_controller_ptr;

TEST(UAVTest, uav_integration_test)
{
  bool result = false;
  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.01).sleep();

    if (uav_test_controller_ptr->isFinished()) {
      result = uav_test_controller_ptr->getResult();
      break;
    }
  }

  EXPECT_TRUE(result);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_integration_test");
  uav_test_controller_ptr = std::make_shared<uav_tests::UAVTestController>();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}