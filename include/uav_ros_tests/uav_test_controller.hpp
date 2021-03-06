#ifndef UAV_TEST_CONTROLLER_HPP
#define UAV_TEST_CONTROLLER_HPP

#include <iostream>

#include <ros/ros.h>
#include <uav_ros_lib/topic_handler.hpp>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
namespace uav_tests {

/**
 * @brief This enum defines states for the UAVTsestController to cycle through.
 *
 */
enum UAVTestState { IDLE, TAKEOFF, POST_TAKEOFF, FINISHED_STATE, ERROR_STATE };

std::ostream &operator<<(std::ostream &os, const UAVTestState &s);

/**
 * @brief Class Used for conducting UAV Integraation tests.
 *
 */
class UAVTestController
{
public:
  explicit UAVTestController(ros::NodeHandle &nh);

  /**
   * @brief Check if test are finished.
   *
   * @return true if tests are finished.
   * @return false if tests are in progress.
   */
  bool isFinished();

  /**
   * @brief Get the test result.
   *
   * @return true If test passed.
   * @return false If test failed.
   */
  bool getResult();

private:
  void change_state(const UAVTestState &new_state);
  void loop_timer(const ros::TimerEvent &event);

  static constexpr auto TEST_TIMEOUT = 600.0;
  static constexpr auto MESSAGE_THROTTLE = 1.0;
  static constexpr auto TAKEOFF_HEIGHT = 2.0;

  ros::NodeHandle m_nh;
  UAVTestState m_curr_state = IDLE;
  ros::Time m_start_time;

  ros::Timer m_loop_timer;
  ros_util::TopicHandler<nav_msgs::Odometry> m_odom_handler;
  ros_util::TopicHandler<nav_msgs::Odometry> m_global_odom_handler;
  ros_util::TopicHandler<mavros_msgs::State> m_mstate_handler;
};
}// namespace uav_tests

#endif /* UAV_TEST_CONTROLLER_HPP */
