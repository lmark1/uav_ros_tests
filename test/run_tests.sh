#!/bin/bash

# Run the uav integration tests
# Credits to https://github.com/ctu-mrs/mrs_uav_testing/blob/master/test/run_tests.sh

# Exit immediatelly if a command exits with a non-zero status
set -e

# Executes a command when DEBUG signal is emitted in this script - should be after every line
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# Executes a command when ERR signal is emmitted in this script
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

PACKAGE="uav_ros_tests"
VERBOSE=1

[ "$VERBOSE" = "0" ] && TEXT_OUTPUT=""
[ "$VERBOSE" = "1" ] && TEXT_OUTPUT="-t"

# build the package
catkin build $PACKAGE # it has to be fully build normally before building with --catkin-make-args tests
catkin build $PACKAGE --catkin-make-args tests

# folder for test results
TEST_RESULT_PATH=$(realpath /tmp/$RANDOM)
mkdir -p $TEST_RESULT_PATH

# run the test
export UAV_NAMESPACE=red
rostest $PACKAGE kopterworx_base_rostest.launch $TEXT_OUTPUT --results-filename=$PACKAGE.test --results-base-dir="$TEST_RESULT_PATH"

# evaluate the test results
catkin_test_results "$TEST_RESULT_PATH"

echo ""
echo "Success? $?"