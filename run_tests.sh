#!/bin/bash
TESTS_DIR=./tests

# This script runs all of the unit tests for the roslaunch-to-dot script.
# In order for the tests to locate the test launch files to use the examples
# directory must be added to the ROS package path prior to executing the
# test script. This helper function takes care of that.
ROS_PACKAGE_PATH=$PWD/examples:$ROS_PACKAGE_PATH python3 $TESTS_DIR/run_tests.py

# Remove the *.pyc files generated from running the tests
find $TESTS_DIR -iname "*.pyc" | xargs rm -f
