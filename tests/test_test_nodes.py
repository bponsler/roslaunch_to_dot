import unittest

from util import roslaunch_to_dot, Color, ErrorMsg


class TestTestNodes(unittest.TestCase):
    def testTestNode(self):
        launchFile = "examples/fake_package/launch/test_node.launch"

        status, output, graph = roslaunch_to_dot(launchFile)

        print "STATUS:", status
        print "OUTPUT:", output

        # Should not have failed
        self.assertEqual(status, 0)
        self.assertIsNotNone(graph)

        # The test node should be included in the graph
        node = graph.get_node("node_fake_package_test_node_a_node")
        self.assertIsNotNone(node)


if __name__ == '__main__':
    unittest.main()
