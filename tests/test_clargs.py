import unittest

from util import roslaunch_to_dot, Color, ErrorMsg


class TestCommandLineArguments(unittest.TestCase):
    ClaNodeName = "command_line_args"

    def testNoCommandLineArguments(self):
        launchFile = "examples/fake_package/launch/example.launch"
        args = []

        status, output, graph = roslaunch_to_dot(launchFile)

        # Should not have failed
        self.assertEqual(status, 0)

        # There should not be a command line arguments node in this case
        with self.assertRaises(Exception):
            graph.get_node(self.ClaNodeName)

    def testCommandLineArguments(self):
        launchFile = "examples/fake_package/launch/example.launch"
        args = [
            "fake:=value",
            "another:=1234",
            "bool:=true",
            "foo:=bar",
        ]

        status, output, graph = roslaunch_to_dot(launchFile, args=args)

        # Should not have failed
        self.assertEqual(status, 0)

        # There must be a command line arguments node in this case
        cla = graph.get_node(self.ClaNodeName)
        self.assertIsNotNone(cla)

        # Check that each of the input command line arguments exists
        # in the label for the node
        claLabel = cla.attr['label']
        for argStr in args:
            self.assertTrue(argStr in claLabel)


if __name__ == '__main__':
    unittest.main()
