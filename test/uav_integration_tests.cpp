#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(UAVTest, uav_integration_test)
{
  ros::Time::init();
  ros::Duration(30.0).sleep();
  EXPECT_TRUE(true);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_integration_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}