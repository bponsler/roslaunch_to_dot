'''
Run this from the roslaunch_to_dot directory using the following command:

   ROS_PACKAGE_PATH=$PWD/examples:$ROS_PACKAGE_PATH python ./tests/run_tests.py

'''
import sys
import inspect
import unittest
from os import environ
from os.path import join, exists

# Import all test classes
from test_arg import TestArg
from test_env import TestEnv
from test_png import TestPng
from test_cycle import TestCycle
from test_optenv import TestOptEnv
from test_package import TestPackage
from test_rosparam import TestRosParam
from test_filetypes import TestFileTypes
from test_clargs import TestCommandLineArguments
from test_show_node_type import TestShowNodeType
from test_disable_groups import TestDisableGroups
from test_nodes_same_name import TestNodesSameName
from test_missing_launch_file import TestMissingLaunchFile

# Check for the ROS environment
try:
    import roslib
except:
    print "ERROR: you must run this from the ROS environment"
    exit(0)


if __name__ == '__main__':
    # Make sure we can find the examples directory
    examplesDir = join(environ.get("PWD"), "examples")
    if not exists(examplesDir):
        print "ERROR: Must be run from the roslaunch_to_dot directory!"
        exit(1)

    # Make sure we can find the ROS package path
    rosPackagePath = environ.get("ROS_PACKAGE_PATH", None)
    if rosPackagePath is None:
        print "ERROR: Cannot find ROS_PACKAGE_PATH!"
        exit(2)

    # Add the examples directory to the ROS package path so that ROS
    # can properly locate the test launch files
    rosPackagePath = rosPackagePath.split(":")

    # Make sure ROS can find the packages in the examples directory
    if examplesDir not in rosPackagePath:
        print "ERROR: The roslaunch_to_dot/examples directory must be in " \
            "your ROS_PACKAGE_PATH!"
        exit(3)

    # Automatically locate all items in the module that are TestCases
    testModules = []
    for itemName in dir():
        item = getattr(sys.modules[__name__], itemName)
        if inspect.isclass(item) and issubclass(item, unittest.TestCase):
            testModules.append(item)

    # Run all of the test cases that have been found
    for testModule in testModules:
        suite = unittest.TestLoader().loadTestsFromTestCase(testModule)
        unittest.TextTestRunner(verbosity=2).run(suite)
