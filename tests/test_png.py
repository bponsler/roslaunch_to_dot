import unittest
from os import remove
from os.path import exists

from util import roslaunch_to_dot, Color, ErrorMsg


class TestPng(unittest.TestCase):
    PngFile = "/tmp/test.png"

    def testNoPng(self):
        launchFile = "examples/fake_package/launch/example.launch"
        options = []

        # Delete the PNG to make sure it's gone before the command
        if exists(self.PngFile):
            remove(self.PngFile)

        status, output, graph = roslaunch_to_dot(
            launchFile, options=options)

        # Should not have failed
        self.assertEqual(status, 0)

        # The PNG should still NOT exist at this point
        self.assertFalse(exists(self.PngFile))

    def testCreatePng(self):
        launchFile = "examples/fake_package/launch/example.launch"
        options = [
            "--png",
            ]

        # Delete the PNG to make sure it's gone before the command
        if exists(self.PngFile):
            remove(self.PngFile)

        status, output, graph = roslaunch_to_dot(
            launchFile, options=options)

        # Should not have failed
        self.assertEqual(status, 0)

        # The PNG SHOULD exist at this point
        self.assertTrue(exists(self.PngFile))


if __name__ == '__main__':
    unittest.main()
