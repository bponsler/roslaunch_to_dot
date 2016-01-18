import unittest

from util import roslaunch_to_dot, Color, ErrorMsg


class TestNodesSameName(unittest.TestCase):
    IdenticalNodeName = "a_node"

    def testNodeSameName(self):
        launchFile = "examples/fake_package/launch/identical_nodes.launch"

        status, output, graph = roslaunch_to_dot(launchFile)

        # Should not have failed
        self.assertEqual(status, 0)

        # Generate the error message string
        errorMsg = ErrorMsg.NodesWithSameName % self.IdenticalNodeName

        # The output should contain a warning indicating that there were
        # two nodes that have the same name
        numErrors = output.count(errorMsg)
        self.assertEqual(numErrors, 1)


if __name__ == '__main__':
    unittest.main()
