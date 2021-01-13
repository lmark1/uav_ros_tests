cmake_minimum_required(VERSION 3.2)
project(uav_ros_tests)
add_compile_options(-Wall -Wextra)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

# ########################################################################################
# Testing ##
# ########################################################################################

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(uav_stack_test test/kopterworx_base_rostest.launch
                    test/uav_integration_tests.cpp)
  target_link_libraries(uav_stack_test ${catkin_LIBRARIES})
  add_dependencies(uav_stack_test ${catkin_EXPORTED_TARGETS})
endif()
