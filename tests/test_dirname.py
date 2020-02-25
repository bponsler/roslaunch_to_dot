import unittest
from os import environ

from util import roslaunch_to_dot, Color, ErrorMsg


class TestDirname(unittest.TestCase):
    EnvVar = "ROBOT"

    def testDirname(self):
        launchFile = "examples/fake_package/launch/dirname.launch"

        status, output, graph = roslaunch_to_dot(launchFile)

        # Should not have failed (the error gets caught and does not propagate
        # to the top level)
        self.assertEqual(status, 0)


if __name__ == '__main__':
    unittest.main()
