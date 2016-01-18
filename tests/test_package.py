import unittest
from os.path import abspath

from util import roslaunch_to_dot, Color, ErrorMsg


class TestPackage(unittest.TestCase):
    def testOrphanPackage(self):
        launchFile = "examples/orphan.launch"

        status, output, graph = roslaunch_to_dot(launchFile)

        # Should definitely have failed
        self.assertNotEqual(status, 0)

        errorMsg = ErrorMsg.FailedToGetPackage % abspath(launchFile)

        # Must generate an error indicating that the package for the file
        # was not able to be found
        numErrors = output.count(errorMsg)
        self.assertEqual(numErrors, 1)


if __name__ == '__main__':
    unittest.main()
