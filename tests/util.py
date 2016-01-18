from os import environ
from os.path import join
from commands import getstatusoutput

try:
    import pygraphviz as gv
except ImportError:
    raise ImportError("Please run 'sudo apt-get install python-pygraphviz'")


# This list must match the one from the script
LAUNCH_FILE_TYPES = [".launch", ".test", ".xml"]


# Various error messages
class ErrorMsg:
    CanNotFindLaunchFile = "ERROR: Can not find launch file: %s"
    CanNotLocateRosPackage = "Cannot locate installation of package %s: " \
                             "[rospack] Error: package"
    Cycle = "ERROR: There is a cycle in the launch file graph from:"
    FailedToGetPackage = "Failed to get package name for: %s"
    MissingArg = "Could not resolve unknown arg: '%s'"
    MissingEnvVar = "Could not find environment variable: '%s'"
    MissingLaunchFile = "WARNING: Could not locate launch file"
    NodesWithSameName = "WARNING: There are two nodes in the launch tree " \
                    "that have the same name: %s"
    InvalidArg = "ERROR: invalid syntax for arg %s: %s"
    InvalidOverride = "WARNING: cannot override arg '%s', which has " \
                      "already been set."
    InvalidFiletype = "Must be given a supported filetype: %s: " % \
                      ', '.join(LAUNCH_FILE_TYPES)


# NOTE: these must match the colors from roslaunch_to_dot
class Color:
    ConditionalLine = "#ff8c00"
    CycleLine = "red"
    DuplicateNode = "red"
    LaunchFile = "#d3d3d3"
    Line = "black"
    MissingFile = "#cc0000"
    Node = "#6495ed"
    TestNode = "#009900"


def roslaunch_to_dot(launchFile, options=None, args=None, prefix=None):
    '''Execute a call to the roslaunch-to-dot script for the given
    launch file.

    Returns a tuple with the following three items:
        - status -- the exit code status of the call to the script
        - output -- the console output of the script
        - graph -- the graphviz graph object, or None if the call failed

    * launchFile -- the launch file to convert to a dot graph
    * options -- the list of additional command line options for the script
    * args -- the list of additional arguments to set for the script
    * prefix -- string to prefix before the command

    '''
    # Get the path to the script
    roslaunch_to_dot = join(environ.get("PWD"), "roslaunch-to-dot.py")

    # Convert the list of options to usable command line arguments
    options = [] if options is None else options
    optionsStr = ' '.join(map(str, options))  # All options must be strings

    # Convert the list of arguments to usable command line arguments
    args = [] if args is None else args
    argsStr = ' '.join(map(str, args))  # All args must be strings

    # Create the command to run
    dotFile = "/tmp/test.dot"  # Use a temp dot file
    command = "%s %s %s %s %s" % \
        (roslaunch_to_dot, optionsStr, launchFile, dotFile, argsStr)

    # Add the command prefix, if one is provided
    if prefix is not None:
        command = "%s %s" % (prefix, command)

    # Run the command and return the output
    status, output = getstatusoutput(command)

    # Load the graph from the file, if the script execution was successful
    graph = None
    if status == 0:
        graph = gv.AGraph(dotFile)

    return status, output, graph
