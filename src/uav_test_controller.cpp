#include <uav_ros_tests/uav_test_controller.hpp>
#include <uav_ros_control/util/uav_util.hpp>
#include <uav_ros_lib/ros_convert.hpp>

std::ostream &uav_tests::operator<<(std::ostream &os, const UAVTestState &s)
{
  switch (s) {
  case IDLE:
    os << "IDLE";
    break;

  case TAKEOFF:
    os << "TAKEOFF";
    break;

  case POST_TAKEOFF:
    os << "POST TAKEOFF";
    break;

  case FINISHED_STATE:
    os << "FINISHED STATE";
    break;

  case ERROR_STATE:
    os << "ERROR STATE";
    break;

  default:
    os << "Unknwon state " << s;
    break;
  }

  return os;
}

uav_tests::UAVTestController::UAVTestController(ros::NodeHandle &nh)
  : m_nh(nh), m_odom_handler(m_nh, "odometry"), m_mstate_handler(m_nh, "mavros/state"),
    m_global_odom_handler(m_nh, "global_odometry"), m_start_time(ros::Time::now())
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

  if (!m_mstate_handler.isMessageRecieved()) {
    ROS_INFO_THROTTLE(MESSAGE_THROTTLE, "[UAVTestController] mavros/state unresponsive.");
    return;
  }

  if (!m_odom_handler.isMessageRecieved()) {
    ROS_INFO_THROTTLE(
      MESSAGE_THROTTLE, "[UAVTestController] odometry topic unresponsive.");
    return;
  }

  auto [odom_x, odom_y, odom_z] =
    ros_convert::extract_odometry(m_global_odom_handler.getData());
  ROS_INFO_STREAM_THROTTLE(MESSAGE_THROTTLE, "[UAVTestController] At " << m_curr_state);
  switch (m_curr_state) {
  case IDLE:
    change_state(TAKEOFF);
    break;

  case TAKEOFF:
    if (uav_util::automatic_takeoff(m_nh, TAKEOFF_HEIGHT)) {
      change_state(POST_TAKEOFF);
    } else {
      ROS_WARN_THROTTLE(MESSAGE_THROTTLE, "[UAVTestController] Takeoff unsuccessful.");
    }
    break;

  case POST_TAKEOFF:

    // If UAV is not armed post takeoff go back to takeoff
    if (!m_mstate_handler.getData().armed) {
      change_state(TAKEOFF);
      break;
    }
    
    // Check UAV altitude
    if (abs(odom_z - TAKEOFF_HEIGHT) < 0.1) {
      ROS_INFO_STREAM("[UAVTestController] Takeoff altitude " << odom_z << " achieved.");
      change_state(FINISHED_STATE);
    } else {
      ROS_WARN_STREAM_THROTTLE(MESSAGE_THROTTLE,
        "[UAVTestController] UAV altitude " << odom_z << " not at takeoff altitude "
                                            << TAKEOFF_HEIGHT);
    }
    break;

  case ERROR_STATE:
    break;

  default:
    ROS_FATAL_STREAM("[UAVTestController] Unable to process state " << m_curr_state);
    change_state(ERROR_STATE);
    break;
  }

  ROS_INFO_STREAM_THROTTLE(5.0,
    "[UAVTestController] UAV odometry: [" << odom_x << ", " << odom_y << ", " << odom_z
                                          << "]");
}

bool uav_tests::UAVTestController::isFinished()
{
  return m_curr_state == FINISHED_STATE || m_curr_state == ERROR_STATE;
}

bool uav_tests::UAVTestController::getResult() { return m_curr_state == FINISHED_STATE; }

void uav_tests::UAVTestController::change_state(const UAVTestState &new_state)
{
  ROS_INFO_STREAM("[UAVTestController] Change to " << new_state);
  switch (new_state) {

  case TAKEOFF:
    m_curr_state = TAKEOFF;
    break;

  case POST_TAKEOFF:
    m_curr_state = POST_TAKEOFF;
    break;

  case FINISHED_STATE:
    ROS_INFO("[UAVTestController] Tests finished!");
    m_curr_state = FINISHED_STATE;
    break;

  case ERROR_STATE:
    ROS_FATAL("[UAVTestController] Test Failed!");
    m_curr_state = ERROR_STATE;
    break;

  default:
    ROS_FATAL_STREAM("[UAVTestController] Unable to change state to " << new_state);
    change_state(ERROR_STATE);
    break;
  }
}