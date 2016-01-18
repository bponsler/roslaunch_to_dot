import unittest

from util import roslaunch_to_dot, Color, ErrorMsg


class TestShowNodeType(unittest.TestCase):
    def testDoNotShowNodeType(self):
        launchFile = "examples/fake_package/launch/example.launch"
        options = []

        status, output, graph = roslaunch_to_dot(
            launchFile, options=options)

        # Should not have failed
        self.assertEqual(status, 0)

        # Ensure that no nodes contain the type of node in their label
        for node in graph.nodes():
            # Only care about ROS nodes
            if node.name.startswith("node_"):
                label = node.attr["label"]

                # Node labels should NOT contain the type of node
                self.assertFalse("type: " in label)

    def testShowNodeType(self):
        launchFile = "examples/fake_package/launch/example.launch"
        options = [
            "--show-node-type",
            ]

        status, output, graph = roslaunch_to_dot(
            launchFile, options=options)

        # Should not have failed
        self.assertEqual(status, 0)

        # Ensure that all nodes contain the type of node in their label
        for node in graph.nodes():
            # Only care about ROS nodes
            if node.name.startswith("node_"):
                label = node.attr["label"]

                # Node label should contain the type of node
                self.assertTrue("type: " in label)


if __name__ == '__main__':
    unittest.main()
