#ifndef UAV_TEST_CONTROLLER_HPP
#define UAV_TEST_CONTROLLER_HPP

#include <ros/ros.h>

namespace uav_tests {

/**
 * @brief This enum defines states for the UAVTsestController to cycle through.
 *
 */
enum UAVTestState { IDLE, TAKEOFF };

/**
 * @brief Class Used for conducting UAV Integraation tests.
 *
 */
class UAVTestController
{
public:
  UAVTestController();

private:
  /**
   * @brief Main event loop timer
   *
   * @param event
   */
  void loop_timer(const ros::TimerEvent &event);

  ros::NodeHandle m_nh;
  UAVTestState m_curr_state = IDLE;
};
}// namespace uav_tests

#endif /* UAV_TEST_CONTROLLER_HPP */