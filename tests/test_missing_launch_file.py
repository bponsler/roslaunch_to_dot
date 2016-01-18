import unittest
from os.path import abspath

from util import roslaunch_to_dot, Color, ErrorMsg


class TestMissingLaunchFile(unittest.TestCase):
    def testMissingLaunchFile(self):
        # Try to access several missing files...
        missingFiles = [
            "this_does_not_exist",
            "whatever",
            "why_is_this_missing",
            ]

        for missingFile in missingFiles:
            launchFile = "examples/fake_package/launch/%s.launch" % missingFile

            status, output, graph = roslaunch_to_dot(launchFile)

            # Should have failed
            self.assertNotEqual(status, 0)

            errorMsg = ErrorMsg.CanNotFindLaunchFile % abspath(launchFile)

            # Should have issues an error about not being able to find the file
            numErrors = output.count(errorMsg)
            self.assertEqual(numErrors, 1)


if __name__ == '__main__':
    unittest.main()
