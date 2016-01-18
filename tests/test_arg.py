import unittest

from util import roslaunch_to_dot, Color, ErrorMsg


class TestArg(unittest.TestCase):
    UndefinedArg = "undefined_arg"
    DefaultArg = "default_arg"
    ValueArg = "value_arg"

    def testArgUndefined(self):
        launchFile = "examples/fake_package/launch/arg_error.launch"

        status, output, graph = roslaunch_to_dot(launchFile)

        # Should not have failed (the error gets caught and does not propagate
        # to the top level)
        self.assertEqual(status, 0)

        # Error should not be able to resolve the undefined argument
        numErrors = output.count(ErrorMsg.MissingArg % self.UndefinedArg)
        self.assertEqual(numErrors, 1)

    def testUndefinedArgSet(self):
        launchFile = "examples/fake_package/launch/arg_error.launch"
        args = [
            "%s:=example" % self.UndefinedArg,
        ]

        status, output, graph = roslaunch_to_dot(launchFile, args=args)

        # Should not have failed (the error gets caught and does not propagate
        # to the top level)
        self.assertEqual(status, 0)

        # Should not be able to find TWO launch files (one for default arg, and
        # one for value arg)
        numMissing = output.count(ErrorMsg.MissingLaunchFile)
        self.assertEqual(numMissing, 2)

        # The example launch file should have been included once
        example = graph.get_node("launch_fake_package_example")
        self.assertIsNotNone(graph)

    def testOverrideDefaultArg(self):
        launchFile = "examples/fake_package/launch/arg_error.launch"
        args = [
            "%s:=example" % self.UndefinedArg,  # Keep this so things work
            "%s:=example" % self.DefaultArg,
        ]

        status, output, graph = roslaunch_to_dot(launchFile, args=args)

        # Should not have failed (the error gets caught and does not propagate
        # to the top level)
        self.assertEqual(status, 0)

        # Should not be able to find one launch files (one for value arg)
        numMissing = output.count(ErrorMsg.MissingLaunchFile)
        self.assertEqual(numMissing, 1)

        # The example launch file should have been included twice
        example = graph.get_node("launch_fake_package_example")
        self.assertIsNotNone(graph)

    def testOverrideValueArg(self):
        launchFile = "examples/fake_package/launch/arg_error.launch"
        args = [
            "%s:=example" % self.UndefinedArg,  # Keep this so things work
            "%s:=example" % self.DefaultArg,  # Keep this so things work
            "%s:=example" % self.ValueArg,  # Keep this so things work
        ]

        # The override for the arg that has its "value" specified should
        # fail because roslaunch would error in this case saying that
        # the argument already has its value specified

        status, output, graph = roslaunch_to_dot(launchFile, args=args)

        # Should not have failed (the error gets caught and does not propagate
        # to the top level)
        self.assertEqual(status, 0)

        # Should not be able to find one launch files (one for value arg)
        numMissing = output.count(ErrorMsg.MissingLaunchFile)
        self.assertEqual(numMissing, 1)

        # Should not be able to override the value of the 'value' arg since
        # its value has already been specified
        numInvalid = output.count(ErrorMsg.InvalidOverride % self.ValueArg)
        self.assertEqual(numInvalid, 1)

        # The example launch file should have been included twice
        example = graph.get_node("launch_fake_package_example")
        self.assertIsNotNone(graph)

    def testInvalidArg(self):
        # Various invalid attempts to specify arguments on the command line
        argAttempts = [
            "%s==example" % self.UndefinedArg,
            "%s=example" % self.UndefinedArg,
            "%s:example" % self.UndefinedArg,
            "%s-example" % self.UndefinedArg,
            ]

        for attempt in argAttempts:
            launchFile = "examples/fake_package/launch/arg_error.launch"
            args = [attempt]

            status, output, graph = roslaunch_to_dot(launchFile, args=args)

            # Should fail -- invalid arg
            self.assertNotEqual(status, 0)

            errorMsg = ErrorMsg.InvalidArg % ("0", attempt)

            # Should have an error about invalid arg
            numInvalid = output.count(errorMsg)
            self.assertEqual(numInvalid, 1)


if __name__ == '__main__':
    unittest.main()
