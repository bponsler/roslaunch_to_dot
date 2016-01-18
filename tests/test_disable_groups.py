import unittest

from util import roslaunch_to_dot, Color, ErrorMsg


class TestDisableGroups(unittest.TestCase):
    def testGroupsEnabled(self):
        launchFile = "examples/fake_package/launch/example.launch"
        options = []

        status, output, graph = roslaunch_to_dot(
            launchFile, options=options)

        # Should not have failed
        self.assertEqual(status, 0)

        # Should have exactly one subgraph
        self.assertEqual(len(graph.subgraphs()), 1)

        # Ensure that no nodes contain the node's package in their label
        for node in graph.nodes():
            # Only care about ROS nodes
            if node.name.startswith("node_"):
                # Node labels should NOT contain the nodes's package
                self.assertFalse("pkg: " in node.attr["label"])

    def testGroupsDisabled(self):
        launchFile = "examples/fake_package/launch/example.launch"
        options = [
            "--disable-groups",
            ]

        status, output, graph = roslaunch_to_dot(
            launchFile, options=options)

        # Should not have failed
        self.assertEqual(status, 0)

        # Should have exactly zero subgraphs
        self.assertEqual(len(graph.subgraphs()), 0)

        # Ensure that all nodes contain the node's package in their label
        for node in graph.nodes():
            # Node label should contain the node's package
            self.assertTrue("pkg: " in node.attr["label"])


if __name__ == '__main__':
    unittest.main()
