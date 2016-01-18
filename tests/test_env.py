import unittest
from os import environ

from util import roslaunch_to_dot, Color, ErrorMsg


class TestEnv(unittest.TestCase):
    EnvVar = "ROBOT"

    def testEnvUndefined(self):
        launchFile = "examples/fake_package/launch/env_error.launch"

        status, output, graph = roslaunch_to_dot(launchFile)

        # Should not have failed (the error gets caught and does not propagate
        # to the top level)
        self.assertEqual(status, 0)

        # Error should not be able to find environment variable
        numErrors = output.count(ErrorMsg.MissingEnvVar % self.EnvVar)
        self.assertEqual(numErrors, 1)

    def testEnvDefined(self):
        launchFile = "examples/fake_package/launch/env_error.launch"

        # Define the environment variable
        prefix = "%s=robot" % self.EnvVar

        status, output, graph = roslaunch_to_dot(launchFile, prefix=prefix)

        # Should not have failed (the error gets caught and does not propagate
        # to the top level)
        self.assertEqual(status, 0)

        # Error should be able to find ROBOT environment variable
        numErrors = output.count(ErrorMsg.MissingEnvVar % self.EnvVar)
        self.assertEqual(numErrors, 0)


if __name__ == '__main__':
    unittest.main()
