import unittest

from util import roslaunch_to_dot, Color, ErrorMsg


class TestCycle(unittest.TestCase):
    def testCycle(self):
        launchFile = "examples/fake_package/launch/example.launch"

        status, output, graph = roslaunch_to_dot(launchFile)

        # Should not have failed
        self.assertEqual(status, 0)

        # The output should contain the cycle error message TWICE:
        #    once for a cycle between cycle.launch and itself, and another
        #    for the cycle between cycle.launch and example.launch
        numErrors = output.count(ErrorMsg.Cycle)
        self.assertEqual(numErrors, 2)

        # Make sure the proper nodes exist
        exampleLaunch = graph.get_node("launch_fake_package_example")
        self.assertIsNotNone(exampleLaunch)

        cycleLaunch = graph.get_node("launch_fake_package_cycle")
        self.assertIsNotNone(cycleLaunch)

        # Make sure the proper edges exist
        cycle2Cycle = graph.get_edge(cycleLaunch, cycleLaunch)
        self.assertIsNotNone(cycle2Cycle)

        cycle2Example = graph.get_edge(cycleLaunch, exampleLaunch)
        self.assertIsNotNone(cycle2Example)

        # Make sure both edges are colored "red"
        self.assertEqual(cycle2Cycle.attr["color"], Color.CycleLine)
        self.assertEqual(cycle2Example.attr["color"], Color.CycleLine)


if __name__ == '__main__':
    unittest.main()
