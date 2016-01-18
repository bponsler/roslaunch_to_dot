import unittest

from util import roslaunch_to_dot, Color, ErrorMsg


class TestRosParam(unittest.TestCase):
    def testNoRosParamNodes(self):
        launchFile = "examples/fake_package/launch/rosparam_nodes.launch"

        status, output, graph = roslaunch_to_dot(launchFile)

        # Should not have failed
        self.assertEqual(status, 0)

        # The graph should not include ANY rosparam nodes
        for node in graph.nodes():
            # The name of all rosparam file nodes starts with yaml_
            self.assertFalse(node.name.startswith("yaml_"))

    def testNoRosParamNodes(self):
        launchFile = "examples/fake_package/launch/rosparam_nodes.launch"
        options = [
            "--show-rosparam-nodes",  # Include rosparam nodes
            ]

        status, output, graph = roslaunch_to_dot(launchFile, options=options)

        # Should not have failed
        self.assertEqual(status, 0)

        # The graph should not include rosparam nodes
        defaults = graph.get_node("yaml_fake_package_defaults_yaml")
        robot = graph.get_node("yaml_fake_package_robot_yaml")

        # Both rosparam files should exist
        self.assertIsNotNone(defaults)
        self.assertIsNotNone(robot)

        # Assert the correct labels on the yaml nodes
        self.assertEqual(defaults.attr["label"], "defaults.yaml")
        self.assertEqual(robot.attr["label"], "robot.yaml")

    def testOverrideRosParamArg(self):
        robotValue = "a_different_robot"

        launchFile = "examples/fake_package/launch/rosparam_nodes.launch"
        options = [
            "--show-rosparam-nodes",  # Include rosparam nodes
            ]
        args = [
            "robot:=%s" % robotValue,
            ]

        status, output, graph = roslaunch_to_dot(
            launchFile, options=options, args=args)

        # Should not have failed
        self.assertEqual(status, 0)

        # The graph should not include rosparam nodes
        defaults = graph.get_node("yaml_fake_package_defaults_yaml")
        robot = graph.get_node("yaml_fake_package_%s_yaml" % robotValue)

        # Both rosparam files should exist
        self.assertIsNotNone(defaults)
        self.assertIsNotNone(robot)

        # Assert the correct labels on the yaml nodes
        self.assertEqual(defaults.attr["label"], "defaults.yaml")
        self.assertEqual(robot.attr["label"], "%s.yaml" % robotValue)

        # The robot rosparam node should be solid colored to indicate
        # that it is missing
        self.assertEqual(robot.attr["color"], Color.MissingFile)
        self.assertEqual(robot.attr["style"], "filled")

    def testRosParamMissingPackage(self):
        package = "this_package_does_not_exist"

        launchFile = \
            "examples/fake_package/launch/rosparam_unknown_pkg.launch"
        options = [
            "--show-rosparam-nodes",  # Include rosparam nodes
            ]
        args = []

        status, output, graph = roslaunch_to_dot(
            launchFile, options=options, args=args)

        # Script does not fail, just generates an error
        self.assertEqual(status, 0)

        # Create the error message string
        errorMsg = ErrorMsg.CanNotLocateRosPackage % package

        # Must have an error
        numErrors = output.count(errorMsg)
        self.assertEqual(numErrors, 1)


if __name__ == '__main__':
    unittest.main()
