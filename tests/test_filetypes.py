import unittest

from util import roslaunch_to_dot, Color, ErrorMsg, LAUNCH_FILE_TYPES


class TestFileTypes(unittest.TestCase):
    def testFileTypes(self):
        # Check all of the filetypes
        for fileType in LAUNCH_FILE_TYPES:
            launchFile = "examples/fake_package/launch/filetype%s" % fileType

            status, output, graph = roslaunch_to_dot(launchFile)

            # Should not have failed
            self.assertEqual(status, 0)

            # No errors present
            numErrors = output.count("ERROR")
            self.assertEqual(numErrors, 0)

        # Test some filetypes that are NOT supported
        for fileType in [".txt", ".lunch"]:  # lunch is correct
            launchFile = "examples/fake_package/launch/filetype%s" % fileType

            status, output, graph = roslaunch_to_dot(launchFile)

            # Should have failed
            self.assertNotEqual(status, 0)

            # Invalid filetype error is present
            numErrors = output.count(ErrorMsg.InvalidFiletype)
            self.assertEqual(numErrors, 1)


if __name__ == '__main__':
    unittest.main()
