import unittest

from util import roslaunch_to_dot, Color, ErrorMsg


class TestOptEnv(unittest.TestCase):
    FirstEnvVar = "FIRST_ROBOT"
    SecondEnvVar = "SECOND_ROBOT"

    def testOptEnvUndefined(self):
        launchFile = "examples/fake_package/launch/optenv_error.launch"

        status, output, graph = roslaunch_to_dot(launchFile)

        # Should have failed (the error gets propagates to the top level)
        self.assertNotEqual(status, 0)

        # Error be able to find first environment variable
        numErrors = output.count(ErrorMsg.MissingEnvVar % self.FirstEnvVar)
        self.assertEqual(numErrors, 0)

        # Should not be able to find the second environment variable
        # (which does not have a default value)
        numErrors = output.count(ErrorMsg.MissingEnvVar % self.SecondEnvVar)
        self.assertEqual(numErrors, 1)

    def testOptEnvDefined(self):
        launchFile = "examples/fake_package/launch/optenv_error.launch"

        # Define the environment variable
        prefix = "%s=robot" % self.SecondEnvVar

        status, output, graph = roslaunch_to_dot(launchFile, prefix=prefix)

        # Should not have failed (the error gets caught and does not propagate
        # to the top level)
        self.assertEqual(status, 0)

        # Shoudl be able to find first environment variable
        numErrors = output.count(ErrorMsg.MissingEnvVar % self.FirstEnvVar)
        self.assertEqual(numErrors, 0)

        # Should be able to find second environment variable
        numErrors = output.count(ErrorMsg.MissingEnvVar % self.SecondEnvVar)
        self.assertEqual(numErrors, 0)

        # Should not be able to locate the launch file that is created using
        # the no default value argument (whose value is the second
        # environment variable)
        numMissing = output.count(ErrorMsg.MissingLaunchFile)
        self.assertEqual(numMissing, 1)


if __name__ == '__main__':
    unittest.main()
