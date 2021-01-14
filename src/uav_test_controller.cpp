#include <uav_ros_tests/uav_test_controller.hpp>

uav_tests::UAVTestController::UAVTestController()
  : m_odom_handler(m_nh, "odometry"), m_start_time(ros::Time::now())
{
  m_loop_timer =
    m_nh.createTimer(ros::Rate(10), &uav_tests::UAVTestController::loop_timer, this);
}

void uav_tests::UAVTestController::loop_timer(
  [[maybe_unused]] const ros::TimerEvent &event)
{  
  if ((ros::Time::now() - m_start_time).toSec() > TEST_TIMEOUT) {
    ROS_FATAL("[UAVTestController] Timeout reached!");
    change_state(ERROR_STATE);
  }

  if (!m_odom_handler.isMessageRecieved()) {
    ROS_INFO_THROTTLE(
      MESSAGE_THROTTLE, "[UAVTestController] odometry topic unresponsive.");
    return;
  }

  ROS_INFO_STREAM_THROTTLE(MESSAGE_THROTTLE, "[UAVTestController] At " << m_curr_state);
  switch (m_curr_state) {
  case IDLE:
    change_state(TAKEOFF);
    break;

  case TAKEOFF:
    break;

  case ERROR_STATE:
    break;

  default:
    ROS_FATAL_STREAM("[UAVTestController] Unable to process state " << m_curr_state);
    change_state(ERROR_STATE);
    break;
  }
}

bool uav_tests::UAVTestController::isFinished()
{
  return m_curr_state == FINISHED_STATE || m_curr_state == ERROR_STATE;
}

bool uav_tests::UAVTestController::getResult() { return m_curr_state == FINISHED_STATE; }

void uav_tests::UAVTestController::change_state(const uav_tests::UAVTestState &new_state)
{
  switch (new_state) {

  case TAKEOFF:
    m_curr_state = TAKEOFF;
    ROS_INFO_STREAM("[UAVTestController] Change to " << m_curr_state);
    break;

  case ERROR_STATE:
    m_curr_state = ERROR_STATE;
    ROS_FATAL("[UAVTestController] Test Failed!");
    break;

  default:
    ROS_FATAL_STREAM("[UAVTestController] Unable to change state to " << new_state);
    change_state(ERROR_STATE);
    break;
  }
}