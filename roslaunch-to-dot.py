#!/usr/bin/env python
'''This script takes a ROS launch file as input and generates a dot graph
file based on the tree of nodes and launch files that will be launched
based on the input launch file.

    $ ./roslaunch-to-dot.py --help
    usage: roslaunch-to-dot.py [-h] [--png] [--show-node-type]
                               [--show-rosparam-nodes]
                               launchFile outputFile [arg [arg ...]]

    Create a dot graph file from a ROS launch file.

    positional arguments:
      launchFile            path to the desired launch file
      outputFile            the output dot file to save
      arg                   override an arg specified anywhere in the launch
                            file tree

    optional arguments:
      -h, --help            show this help message and exit
      --png                 automatically convert the dot file to a PNG
      --disable-groups      don't group nodes/launch files based on their
                            package
      --show-node-type      label ROS nodes with their type in addition to
                            thier name
      --show-rosparam-nodes
                            display nodes and connections for all rosparam
                            files used

'''
import re
import traceback
from sys import argv
from copy import deepcopy
from random import randint
from datetime import datetime
from os import system, environ
from argparse import ArgumentParser
from collections import namedtuple
import xml.etree.ElementTree as ET
from os.path import abspath, exists, basename, splitext, sep

import roslib
import rospkg

try:
    import pygraphviz as gv
except ImportError:
    raise ImportError("Please run 'sudo apt-get install python-pygraphviz'")


# Keep track of a global set of launch files that have already been
# visited so that we can protect ourselves from entering into an
# infinite loop of launch files if there happens to be a recursive
# cycle in the graph
VISITED_LAUNCH_FILES = set()


# Create a named tuple to store attributes pertaining to a node
Node = namedtuple("Node", [
    "launchFile",  # The launch file that contains this node
    "package",  # The name of the ROS package containing this node
    "nodeType",  # The type of ROS node this is
    "name",  # The name of the ROS node
    "dotNodeName",  # The name for the corresponding dot node
    "isTestNode"])  # True if this is a test node, False otherwise

# Create a named tuple to store attributes pertaining to a rosparam file
RosParam = namedtuple("RosParamFile", [
    "filename",   # The resolved filename for the rosparam file
    "name",   # The base name for the rosparam file
    "dotNodeName",  # The name for the corresponding dot node
    "package",  # The package that contains the rosparam file
    "argSubs",  # The dictionary of argument substitutions needed for the file
    ])

# Create a named tuple to store items contained in a single ROS package
PackageItems = namedtuple("PackageItems", [
    "launchFiles",  # The list of launch files contained in this package
    "nodes",  # The list of nodes contains in this package
    "rosParamFiles",  # The list of rosparam files contained in this package
    ])


class LaunchFile:
    '''The LaunchFile class encapsulates a single ROS launch file. This
    class is responsible for parsing the launch file XML and keeping track
    of nodes and launch files that are included within the launch file. It
    is also responsible for properly resolving any ROS launch substitution
    arguments that may exist within strings that it uses within the launch
    file.

    In addition to this, this class is capable of producing the text to
    export this launch file (and all of the nodes and other launch files
    it connects to) into a dot graph.

    '''

    # Identifiers used as elements within a launch file
    ArgTag = "arg"
    GroupTag = "group"
    IncludeTag = "include"
    NodeTag = "node"
    RosParamTag = "rosparam"
    TestTag = "test"

    # Identifiers used as element attribute names within a launch file
    CommandAttribute = "command"
    DefaultAttribute = "default"
    FileAttribute = "file"
    IfAttribute = "if"
    NameAttribute = "name"
    PkgAttribute = "pkg"
    TestNameAttribute = "test-name"
    TypeAttribute = "type"
    UnlessAttribute = "unless"
    ValueAttribute = "value"

    # Identifiers used as substitution arguments within a launch
    # file, e.g,. $(find package)
    AnonSubstitutionArg = "anon"
    ArgSubstitutionArg = "arg"
    EnvSubstitutionArg = "env"
    FindSubstitutionArg = "find"
    OptEnvSubstitutionArg = "optenv"

    # Identifiers for various rosparam commands
    DumpCommand = "dump"
    LoadCommand = "load"

    # Colors used within the dot graph
    ConditionalLineColor = "#ff8c00"
    CycleLineColor = "red"
    DuplicateNodeColor = "red"
    LaunchFileColor = "#d3d3d3"
    LineColor = "black"
    MissingFileColor = "#cc0000"
    NodeColor = "#6495ed"
    TestNodeColor = "#009900"

    def __init__(self,
                 args,
                 filename,
                 includeArgs=None,
                 overrideArgs=None,
                 ancestors=None):
        '''
        * args -- command line arguments
        * filename -- the ROS launch file
        * includeArgs -- dictionary of arg substitution name value pairs
                         that were used to resolve the name of this launch file
        * overrideArgs -- dictionary of arguments that override any
                          arguments specified in this launch file
        * ancestors -- List of launch files which are ancestors of
                       this launch file

        '''
        self.__inputArgs = args
        self.__ancestors = [] if ancestors is None else deepcopy(ancestors)

        # Include ourself as an ancestor, since references to ourself would
        # also constitute a cycle
        self.__ancestors.append(filename)

        # Cannot use dictionary in default argument because the same
        # object will get reused
        self.__includeArgs = {} if includeArgs is None else includeArgs
        self.__overrideArgs = {} if overrideArgs is None else overrideArgs

        # Determine if this launch file has been parsed before
        hasVisited = (filename in VISITED_LAUNCH_FILES)

        self.__filename = filename
        VISITED_LAUNCH_FILES.add(filename)

        # Check if the filename actually exists
        self.__missing = (not exists(self.__filename))

        # Dictionary of args defined in the launch file mapping arg name to
        # its resolved value
        self.__args = {}

        # Map launch file substitution arguments (e.g, 'find', 'arg') to the
        # function that handle resolving the substitution argument
        self.__substitutionArgFnMap = {
            self.FindSubstitutionArg: self.__onFindSubstitutionArg,
            self.ArgSubstitutionArg: self.__onArgSubstitutionArg,
            self.EnvSubstitutionArg: self.__onEnvSubstitutionArg,
            self.OptEnvSubstitutionArg: self.__onEnvSubstitutionArg,
            self.AnonSubstitutionArg: self.__onAnonSubstitutionArg,
        }

        # List of launch file objects which are included by the launch file
        self.__includes = []

        # Create a list of launch filenames that cycles back to previously
        # visited launch files so that these cycles can be differentiated
        # in the graph
        self.__cycles = []

        # List of Node namedtuple objects associated with this launch file
        self.__nodes = []

        # List of RosParam namedtuplem objects included by this launch file
        self.__rosParamFiles = []

        # Protect against cycles in the launch files
        if not hasVisited:
            #### Only parse the file when it has not been visited before
            if not self.__missing:
                # Only parse if the file exists
                self.__parseLaunchFile(filename)
            else:
                print "WARNING: Could not locate launch " \
                    "file: %s" % self.__filename

    #### Getter functions

    def getFilename(self):
        '''Get the filename for this launch file.'''
        return self.__filename

    def isMissing(self):
        '''Determine if this launch file is missing or not.'''
        return self.__missing

    def getNodes(self):
        '''Get all the nodes included by this launch file.'''
        return self.__nodes

    def getCycles(self):
        '''Return the list of launch file names that are included by this
        launch file but are cycles back to previously visited launch files.

        '''
        return self.__cycles

    def getCleanName(self):
        '''Get the clean (no periods) name for this launch file.'''
        return splitext(basename(self.__filename))[0].replace(".", "_")

    def getDotNodeName(self):
        '''Get the name of the dot node corresponding to this launch file.'''
        cleanName = self.getCleanName()
        packageName = self.getPackageName()

        return "launch_%s_%s" % (packageName, cleanName)

    def getPackageName(self):
        '''Get the name of the package that contains this launch file.'''
        packageName = rospkg.get_package_name(self.__filename)
        if not packageName:
            raise Exception("Failed to get package name for: %s" %
                            self.__filename)

        return packageName

    def getAllLaunchFiles(self):
        '''Get the entire list of launch files included because of
        this launch file.

        '''
        launchFiles = [self]  # Add ourself

        # Recursively add all of our children
        for launchFile in self.__includes:
            launchFiles.extend(launchFile.getAllLaunchFiles())

        return launchFiles

    def getAllNodes(self):
        '''Get all of the nodes that will be launched because of this launch
        file and any launch files it includes.

        '''
        allNodes = []

        # Add our own nodes
        for node in self.__nodes:
            allNodes.append(node)

        # Recursively add remaining nodes
        for include in self.__includes:
            allNodes.extend(include.getAllNodes())

        return allNodes

    def getAllRosParamFiles(self):
        '''Get the entire list of rosparam files included because of
        this launch file.

        '''
        rosParamFiles = []

        # Add our own rosparam files
        for rosParam in self.__rosParamFiles:
            rosParamFiles.append(rosParam)

        # Recursively add all of our children's rosparam files
        for launchFile in self.__includes:
            rosParamFiles.extend(launchFile.getAllRosParamFiles())

        return rosParamFiles

    def getIncludeMap(self):
        '''Return the dictionary mapping launch filenames to the list
        of launch files that they include.

        '''
        includeMap = {}

        # Include a mapping for my own included launch files
        includeMap[self.__filename] = self.__includes

        # Add mappings for each of the subchildren
        for childLaunch in self.__includes:
            childMappings = childLaunch.getIncludeMap()

            # Join the two dictionaries together
            includeMap = dict(includeMap.items() + childMappings.items())

        return includeMap

    def getPackageMap(self):
        '''Get a dictionary mapping names of ROS packages to a tuple
        where the first item in the tuple is the list of launch files included
        in that ROS package, and the second item in the tuple is the list of
        ROS nodes included from that ROS package.

        '''
        # Grab the list of all launch files
        allLaunchFiles = self.getAllLaunchFiles()

        packageMap = {}

        ########################################
        # Add all launch files to their respective packages
        for launchFile in allLaunchFiles:
            packageName = launchFile.getPackageName()
            items = packageMap.get(packageName, PackageItems([], [], []))
            items.launchFiles.append(launchFile)
            packageMap[packageName] = items

        ########################################
        # Add all nodes to their respective packages
        for node in self.getAllNodes():
            items = packageMap.get(node.package, PackageItems([], [], []))
            items.nodes.append(node)
            packageMap[node.package] = items

        ########################################
        # Add all rosparam files to their respective packages
        for rosParam in self.getAllRosParamFiles():
            items = packageMap.get(rosParam.package, PackageItems([], [], []))
            items.rosParamFiles.append(rosParam)
            packageMap[rosParam.package] = items

        return packageMap

    #### Dot graph functions

    def toDot(self):
        '''Return the graph that represents this launch file tree.'''

        # Grab the map of all packages, nodes, and include files
        # used by this launch tree
        packageMap = self.getPackageMap()

        graph = gv.AGraph(
            name=self.getCleanName(),
            strict=False,
            directed=True,
            fontsize=35,
            ranksep=2,
            nodesep=2,
            compound=True)
        graph.node_attr.update(fontsize="35")

        #### Create a subgraph for every known package
        self.__clusterNum = 0
        allNodeNames = set()  # Set of node names to check for duplicates
        for packageName, packageItems in packageMap.iteritems():
            self.__createPackageSubgraph(
                graph, packageName, packageItems, allNodeNames)

        #### Create connections between all launch files

        # Iterate over all packages contained in the launch tree
        for _, packageItems in packageMap.iteritems():
            # Iterate over all launch files in this package
            for launchFile in packageItems.launchFiles:
                parentNodeName = launchFile.getDotNodeName()

                # Grab the list of cycles for this launch file
                cycles = launchFile.getCycles()

                # Iterate over all launch files included by the
                # current launch file
                for include in launchFile.__includes:
                    includeFilename = include.getFilename()
                    includeNodeName = include.getDotNodeName()

                    # Determine if this include is a cycle to a previously
                    # visited node
                    isCycle = (includeFilename in cycles)

                    # Select a color depending on if this is a standard
                    # connection between launch files, or a cycle to a
                    # previously parsed launch file
                    color = self.CycleLineColor if isCycle else self.LineColor

                    label = ""

                    # Grab the set of arg substitutions used to conditionally
                    # include the launch file so that the edge can be labeled
                    # and styled accordingly
                    argSubs = include.__includeArgs
                    if len(argSubs) > 0:
                        # Change the color of the line to indicate that it
                        # required arg substitutions
                        color = self.ConditionalLineColor

                        # Convert all arg name value pairs into a single
                        # string, e.g., given {"one": "two", "three": "four"}
                        # the resulting string should be:
                        #    "one:=two\nthree:=four"
                        # So that each arg pair is on its own line
                        # for improved readability
                        label = '\n'.join(map(
                            lambda t: ":=".join(map(str, t)), argSubs.items()))

                    graph.add_edge(
                        parentNodeName,
                        includeNodeName,
                        label=label,
                        penwidth="3",
                        color=color)

        #### Create connections between launch files and nodes
        for _, packageItems in packageMap.iteritems():
            # Iterate over the nodes in this package
            for node in packageItems.nodes:
                # Grab the dot node name of the launch file for this node
                launchNodeName = node.launchFile.getDotNodeName()

                graph.add_edge(
                    launchNodeName,
                    node.dotNodeName,
                    penwidth=3)

        #### Create connections between launch files and rosparam files
        if self.__inputArgs.showRosParamNodes:
            # Iterate over all packages contained in the launch tree
            for _, packageItems in packageMap.iteritems():
                # Iterate over all launch files in this package
                for launchFile in packageItems.launchFiles:
                    launchNodeName = launchFile.getDotNodeName()

                    # Iterate over all rosparam files needed by the launch file
                    for rosParam in launchFile.__rosParamFiles:
                        # Default attributes
                        color = self.LineColor
                        label = ""

                        # Grab the set of arg substitutions used to
                        # conditionally include the launch file so that the
                        # edge can be labeled and styled accordingly
                        argSubs = rosParam.argSubs
                        if len(argSubs) > 0:
                            # Change the color of the line to indicate that it
                            # required arg substitutions
                            color = self.ConditionalLineColor

                            # Convert all arg name value pairs into a single
                            # string, e.g., given {"one": "2", "three": "4"}
                            # the resulting string should be:
                            #    "one:=2\nthree:=4"
                            # So that each arg pair is on its own line
                            # for improved readability
                            label = '\n'.join(map(
                                lambda t: ":=".join(map(str, t)),
                                argSubs.items()))

                        graph.add_edge(
                            launchNodeName,
                            rosParam.dotNodeName,
                            label=label,
                            penwidth="3",
                            color=color)

        return graph

    def __createPackageSubgraph(self,
                                graph,
                                packageName,
                                packageItems,
                                allNodeNames):
        '''Create a subgraph for a single ROS package.

        * packageName -- the name of the ROS package
        * packageItems -- The PackageItems object
        * allNodeNames -- the set of node names that have already been found so
                          that duplicate nodes can be highlighted

        '''
        subgraphNodes = []

        #### Add one node per launch file contained within this package
        for launchFile in packageItems.launchFiles:
            baseFilename = basename(launchFile.getFilename())
            launchNodeName = launchFile.getDotNodeName()

            # Select the color based on whether or not the file is missing
            color = self.MissingFileColor if launchFile.isMissing() else \
                    self.LaunchFileColor

            label = baseFilename
            if self.__inputArgs.disableGroups:
                #### Include the package name if groups are disabled
                label = label + "\npkg: " + packageName

            # Add a node for each launch file
            graph.add_node(
                launchNodeName,
                label=label,
                shape="rectangle",
                style="filled",
                fillcolor=color)

            # Support disabling subgraph grouping
            if not self.__inputArgs.disableGroups:
                subgraphNodes.append(launchNodeName)

        #### Add one node per node contained within this package
        for node in packageItems.nodes:
            # Change the color to indicate that this is a test node
            color = self.TestNodeColor if node.isTestNode else \
                    self.NodeColor

            # ROS nodes must have unique names, thus alert the user if
            # there are two nodes that have the same name
            if node.name in allNodeNames:
                print "WARNING: There are two nodes in the launch tree " \
                    "that have the same name: %s" % node.name

                # Modify the style of the node if it is a duplicate
                color = self.DuplicateNodeColor

            allNodeNames.add(node.name)

            label = node.name
            if self.__inputArgs.disableGroups:
                #### Include the package name if groups are disabled
                label = label + "\npkg: " + packageName

            # Create the label for the node
            if self.__inputArgs.showNodeType:
                #### Include the node type in addition to its name
                label = label + "\ntype: " + node.nodeType

            ## Add a node for each node
            graph.add_node(
                node.dotNodeName,
                label=label,
                shape="rectangle",
                style="filled",
                fillcolor=color)

            # Support disabling subgraph grouping
            if not self.__inputArgs.disableGroups:
                subgraphNodes.append(node.dotNodeName)

        #### Add one node per rosparam file contained in this package
        if self.__inputArgs.showRosParamNodes:
            for rosParam in packageItems.rosParamFiles:
                # Add the package name when groups are disabled
                label = rosParam.name
                if self.__inputArgs.disableGroups:
                    label = label + "\npkg: " + rosParam.package

                graph.add_node(
                    rosParam.dotNodeName,
                    label=label)

                # Support disabling subgraph grouping
                if not self.__inputArgs.disableGroups:
                    subgraphNodes.append(rosParam.dotNodeName)

        # Support disabling subgraph grouping
        if not self.__inputArgs.disableGroups:
            graph.add_subgraph(
                nbunch=subgraphNodes,
                name="cluster_" + packageName,
                label=packageName,
                penwidth=5)

    ##### Launch file XML parsing functions
    #

    def __parseLaunchFile(self, filename):
        '''Parse a single launch file.

        * filename -- the launch file

        '''
        try:
            tree = ET.parse(filename)
            root = tree.getroot()
        except Exception, e:
            raise Exception(
                "Error while parsing launch file: %s: %s" % (filename, e))

        # Parse all of the launch elements. The XML is parsed serially, meaning
        # that if an argument is used before it is defined then it will
        # generate and error
        self.__parseLaunchElements(root)

    def __parseLaunchElements(self, root):
        '''Parse all of the launch file elements to find other launch files as
        well as ROS nodes contained within the launch file.

        * root -- the launch file XML element

        '''
        # Now we can load all include tags and nodes because we have all of the
        # potential arguments we need
        for child in root:
            # Handle all types of tags
            if child.tag == self.ArgTag:
                # Parse the argument
                self.__parseArgTag(child)
            elif child.tag == self.IncludeTag:
                try:
                    launchFile = self.__parseIncludeTag(child)
                except:
                    traceback.print_exc()
                    continue  # Ignore error
                else:
                    if launchFile is not None:
                        self.__includes.append(launchFile)
            elif child.tag == self.GroupTag:
                try:
                    self.__parseGroupTag(child)
                except:
                    traceback.print_exc()
                    continue  # Ignore error
            elif child.tag == self.NodeTag:
                try:
                    node = self.__parseNodeTag(child)
                except:
                    traceback.print_exc()
                    continue  # Ignore error
                else:
                    # Node is disabled (i.e., if=false, or unless=true)
                    if node is not None:
                        self.__nodes.append(node)
            elif child.tag == self.RosParamTag:
                try:
                    self.__parseRosParam(child)
                except:
                    traceback.print_exc()
                    continue  # Ignore error
            elif child.tag == self.TestTag:
                try:
                    testNode = self.__parseTestNodeTag(child)
                except:
                    traceback.print_exc()
                    continue  # Ignore error
                else:
                    # Test node is disabled (i.e., if=false, or unless=true)
                    if testNode is not None:
                        self.__nodes.append(testNode)

    def __parseArgTag(self, arg):
        '''Parse the argument tag from a launch file.

        * arg -- the argument tag

        '''
        name, value = self.__parseArg(arg)

        # Store the argument -- if it is defined
        if value is not None:
            self.__args[name] = value

            # Determine if the 'value' attribute was specified for this
            # argument, if it was then we do not want to allow it to
            # be overriden
            valueSpecified = (self.ValueAttribute in arg.attrib)
            if valueSpecified and name in self.__overrideArgs:
                del self.__overrideArgs[name]  # Remove it from the override

                print "WARNING: cannot override arg '%s', which has " \
                    "already been set." % name

    def __parseIncludeTag(self, include):
        '''Parse the include tag from a launch file.

        * include -- the include tag

        '''
        # Make sure the include is enabled before continuing
        if not self.__isEnabled(include):
            return None  # Node is disabled

        filename = self.__getAttribute(include, self.FileAttribute)
        if filename is None:
            raise Exception(
                "Include tag missing %s attribute" % self.FileAttribute)

        #### We have another launch file to resolve and parse
        #

        # Resolve the full path to the include file
        resolved = self.__resolveText(filename)

        # If the filename contained any arg substitutions then we want to
        # label the edge indicating the arg and value that was used to
        # conditionally select this launch file
        argSubs = {}  # The file does not use any arg substitutions
        if resolved != filename:
            # Get the dictionary of arg substitutions that this
            # filename contains
            argSubs = self.__getSubstitutionArgs(filename)

        # Protect against cycles in the launch file graph which occurs when a
        # launch file includes a launch file directly in its own ancestor tree
        hasVisited = (resolved in self.__ancestors)
        if hasVisited:
            print "ERROR: There is a cycle in the launch file " \
                "graph from: '%s' to '%s'" % (self.__filename, resolved)
            self.__cycles.append(resolved)  # Add the filename

        # Iterate over all children of the include tag to determine if we
        # are expected to pass any args to the included launch file.
        inheritedArgs = {}
        for child in include:
            if child.tag == self.ArgTag:
                #### Found an arg that should be inherited

                # Grab (and resolve) the name and value of the arg
                name, value = self.__parseArg(child)

                # Allow the child launch file to inherit this argument
                if value is not None:
                    inheritedArgs[name] = value

        # Check for rosparams specified under the include tag
        self.__findRosParams(include)

        # Create the new launch file and parse it -- pass the argument
        # overrides to the child launch file to override the argument anywhere
        # it is defined
        return LaunchFile(
            self.__inputArgs,
            resolved,
            includeArgs=argSubs,
            overrideArgs=inheritedArgs,
            ancestors=self.__ancestors)

    def __parseNodeTag(self, node):
        '''Parse the node tag from a launch file.

        * node -- the node tag

        '''
        # Make sure the node is enabled before continuing
        if not self.__isEnabled(node):
            return None  # Node is disabled

        # Grab all of the node attributes
        pkg = self.__getAttribute(node, self.PkgAttribute)
        nodeType = self.__getAttribute(node, self.TypeAttribute)
        name = self.__getAttribute(node, self.NameAttribute)

        # Any of these attributes may have substitution arguments
        # that need to be resolved
        pkg = self.__resolveText(pkg)
        nodeType = self.__resolveText(nodeType)
        name = self.__resolveText(name)

        # Check for rosparams specified under the node tag
        self.__findRosParams(node)

        # Name for the dot node that will represent this ROS node
        # use the package, node type, and node name for the dot node name
        # to make it fully unique
        dotNodeName = "node_%s_%s_%s" % (pkg, nodeType, name)

        # False means this is not a test node
        return Node(self, pkg, nodeType, name, dotNodeName, False)

    def __findRosParams(self, element):
        '''Find any and all rosparam elements specified under the given
        element.

        * element -- the XML element that may contain rosparam elements

        '''
        # Iterate over children looking for rosparams
        for child in element:
            if child.tag == self.RosParamTag:
                try:
                    self.__parseRosParam(child)
                except:
                    traceback.print_exc()
                    print "WARNING: parsing rosparam"
                    continue  # Ignore error

    def __parseRosParam(self, rosparam):
        '''Parse the rosparam tag from a launch file.

        * rosparam -- the rosparam tag

        '''
        # Load is the default command if it is not provided
        command = rosparam.attrib.get(self.CommandAttribute, self.LoadCommand)

        # The file attribute is only valid for load and dump commands
        if command in [self.LoadCommand, self.DumpCommand]:
            paramFile = rosparam.attrib.get(self.FileAttribute, None)
            if paramFile is not None:
                # Resolve the path to the included file
                resolved = self.__resolveText(paramFile)

                # If the filename contained any arg substitutions then we
                # want to label the edge indicating the arg and value that
                # was used to conditionally select this rosparam file
                argSubs = {}  # The file does not use any arg substitutions
                if resolved != paramFile:
                    # Get the dictionary of arg substitutions that
                    # this filename contains
                    argSubs = self.__getSubstitutionArgs(paramFile)

                # Determine what ROS package contains the rosparam file
                package = rospkg.get_package_name(resolved)
                if package is None:
                    print "ERROR: Failed to find package for rosparam " \
                        "file: %s" % resolved
                    return  # Skip the file

                # Create a unique name for the dot node
                name = basename(resolved)
                cleanName = name.replace(".", "_")
                dotNodeName = "yaml_%s_%s" % (package, cleanName)

                param = RosParam(resolved, name, dotNodeName, package, argSubs)
                self.__rosParamFiles.append(param)

    def __parseTestNodeTag(self, testNode):
        '''Parse the test tag from a launch file.

        * testNode -- the testNode tag

        '''
        # Make sure the test node is enabled before continuing
        if not self.__isEnabled(testNode):
            return None  # Test node is disabled

        # Grab all of the test node attributes
        pkg = self.__getAttribute(testNode, self.PkgAttribute)
        nodeType = self.__getAttribute(testNode, self.TypeAttribute)
        name = self.__getAttribute(testNode, self.TestNameAttribute)

        # Any of these attributes may have substitution arguments
        # that need to be resolved
        pkg = self.__resolveText(pkg)
        nodeType = self.__resolveText(nodeType)
        name = self.__resolveText(name)

        # Name for the dot node that will represent this ROS node
        # use the package, node type, and node name for the dot node name
        # to make it fully unique
        dotNodeName = "node_%s_%s_%s" % (pkg, nodeType, name)

        # True means this is a test node
        return Node(self, pkg, nodeType, name, dotNodeName, True)

    def __parseGroupTag(self, group):
        '''Parse the group tag from a launch file.

        * group -- the group tag

        '''
        # Make sure the group is enabled before continuing
        if not self.__isEnabled(group):
            return None  # Node is disabled

        self.__parseLaunchElements(group)

    def __parseArg(self, arg):
        '''Parse the given arg element to get (and resolve) its name
        and value.

        * arg -- the arg element

        '''
        name = self.__getAttribute(arg, self.NameAttribute)

        # Grab the default and standard value
        default = arg.attrib.get(self.DefaultAttribute, None)
        value = arg.attrib.get(self.ValueAttribute, default)

        # Any of these attributes may have substitution arguments
        # that need to be resolved
        name = self.__resolveText(name)

        # Only resolve the value if it is defined
        if value is not None:
            value = self.__resolveText(value)

        return name, value

    ##### ROS launch substitution argument related functions

    def __resolveText(self, text):
        '''Resolve all of the ROS launch substitution argument
        contained within  given text, e.g.,

        <launch>
            <arg name="example" default="hello" />
            <include file="$(find my_package)/launch/$(arg example)" />
        </launch>

        The string: "$(find my_package)/launch/$(arg example).launch" would
        resolve to:

            /path/to/ros/my_package/launch/hello.launch

        "$(find my_package)" was substituted with the path to "my_package" and
        "$(arg example)" was substituted with the value of the argument named
        "example".

        * text -- the text to resolve

        '''
        # Include files can contain substitution arguments that need to be
        # resolved, e.g.,:
        #    $(find package)/launch/file.launch
        #    $(find package)/launch/$(arg camera).launch
        pattern = re.compile("\$\(([a-zA-Z_]+) ([a-zA-Z0-9_! ]+)\)")

        # Continue until all substitution arguments in the text
        # have been resolved
        results = pattern.search(text)
        while results is not None:
            fullText = results.group()
            subArg, argument = results.groups()

            # Grab the function to handle this specific substitution argument
            substitutionFn = self.__substitutionArgFnMap.get(subArg, None)
            if substitutionFn is None:
                raise Exception(
                    "Include has unknown substitution argument %s" % subArg)

            # Attempt to resolve the substitution argument
            resolved = substitutionFn(argument)

            # Update the text with the value of the
            # resolved substitution argument
            text = text.replace(fullText, resolved)

            # Check for another substitution
            results = pattern.search(text)

        return text

    def __onAnonSubstitutionArg(self, name):
        '''Handle the ROS launch 'anon' substitution argument which aims to
        substitute a randomly generated number inside of some text.

        * name -- the name to anonymize

        '''
        # Just return the given name with a random integer attached
        return "%s-%s" % (name, randint(0, 999))

    def __onArgSubstitutionArg(self, arg):
        '''Handle the ROS launch 'arg' substitution argument which aims to
        substitute the value of a launch file argument inside of some text.

        * package -- the package to find

        '''
        # If the argument is specified in the dictionary of argument
        # overrides, then use the override value
        if arg in self.__overrideArgs:
            return self.__overrideArgs[arg]

        # No override found, use the normal argument
        if arg not in self.__args:
            raise Exception("Could not resolve unknown arg: '%s'" % arg)
        return self.__args[arg]

    def __onEnvSubstitutionArg(self, env):
        '''Handle the ROS launch 'env' or 'optenv' substitution argument which
        aims to substitute the value of an environment variable inside of
        some text.

        * package -- the package to find

        '''
        # Determine if a default value was supplied
        parts = env.split(" ")
        if len(parts) == 1:
            #### No default value was supplied
            if parts[0] not in environ:
                raise Exception(
                    "Could not find environment variable: '%s'" % env)
            return environ[env]
        else:
            #### A default value was supplied
            env, default = parts
            return environ.get(env, default)

    def __onFindSubstitutionArg(self, package):
        '''Handle the ROS launch 'find' substitution argument which aims to
        find the path to a specific ROS package.

        * package -- the ROS package to find

        '''
        return roslib.packages.get_pkg_dir(package)

    def __getSubstitutionArgs(self, text):
        '''Return a dictionary mapping arg names to values for all
        arg substitutions defined in the given text.

        * text -- the given text

        '''
        # Regular expression to find an arg substitution within text:
        #    $(find package)/launch/$(arg camera).launch
        pattern = re.compile("\$\(arg ([a-zA-Z0-9_]+)\)")

        # Name value pair for all arg substitutions in the text
        argSubs = {}

        # Continue until all substitution arguments in the text
        # have been found
        results = pattern.search(text)
        while results is not None:
            fullText = results.group()
            name = results.groups()[0]  # Only a single group

            # Look up the value of the argument (this will raise an
            # Exception if the arg cannot be found)
            value = self.__onArgSubstitutionArg(name)

            # Store the argument for the return value
            argSubs[name] = value

            # Remove substitution argument and continue looking for others
            text = text.replace(fullText, "")

            # Check for another substitution
            results = pattern.search(text)

        return argSubs

    ##### Private helper functions

    def __isEnabled(self, element):
        '''Determine if a ROS launch element is enabled based on the
        value of the 'if' or 'unless' attributes.

        The values for the 'if' and 'unless' atrributes can only be:
            - "true" or "1", or
            - "false" or "0"

        Any other values and this function will raise an Exception.

        * element -- the ROS launch element

        '''
        # Handle the 'if' argument
        ifCase = element.attrib.get(self.IfAttribute, None)
        if ifCase is not None:
            ifCase = self.__resolveText(ifCase)
            if ifCase.lower() in ["false", "0"]:
                return False  # Element is disabled
            elif ifCase.lower() not in ["true", "1"]:
                raise Exception("Invalid value in if attribute: %s" % ifCase)

        # Handle the 'unless' argument
        unlessCase = element.attrib.get(self.UnlessAttribute, None)
        if unlessCase is not None:
            unlessCase = self.__resolveText(unlessCase)
            if unlessCase.lower() in ["true", "1"]:
                return False  # Element is disabled
            elif unlessCase.lower() not in ["false", "0"]:
                raise Exception(
                    "Invalid value in unless attribute: %s" % unlessCase)

        return True  # Element is enabled

    def __getAttribute(self, element, attribute, default=None):
        '''Get an attribute from the given ROS launch XML element. If
        the value does not exist, and the default value given is None
        (its default value), then this function will raise an Exception.

        * element -- the ROS launch element
        * attribute -- the name of the attribute to get
        * default -- the default value

        '''
        value = element.attrib.get(attribute, default)
        if value is None:
            raise Exception("Missing the %s attribute" % attribute)
        return value


if __name__ == '__main__':
    ##### Support various command line arguments
    parser = ArgumentParser(
        description='Create a dot graph file from a ROS launch file.')
    parser.add_argument(
        'launchFile', type=str,
        help='path to the desired launch file')
    parser.add_argument(
        'outputFile',
        help='the output dot file to save')
    parser.add_argument(
        "--png", dest="convertToPng",
        action="store_true", default=False,
        help="automatically convert the dot file to a PNG")
    parser.add_argument(
        "--disable-groups", dest="disableGroups",
        action="store_true", default=False,
        help="don't group nodes/launch files based on their package")
    parser.add_argument(
        "--show-node-type", dest="showNodeType",
        action="store_true", default=False,
        help="label ROS nodes with their type in addition to their name")
    parser.add_argument(
        "--show-rosparam-nodes", dest="showRosParamNodes",
        action="store_true", default=False,
        help="display nodes and connections for all rosparam files used")
    parser.add_argument(
        'overrideArguments', metavar='arg', type=str, nargs='*',
        help='override an arg specified anywhere in the launch file tree')

    # Parse the command line options
    args = parser.parse_args()

    # Grab command line arguments
    launchFile = abspath(args.launchFile)
    dotFilename = args.outputFile

    # Convert the override arguments into a dictionary
    # mapping the argument name to its value
    overrideArgs = {}
    for index, argStr in enumerate(args.overrideArguments):
        # Each argument should be specified using roslaunch style
        #     i.e., NAME:=VALUE
        parts = argStr.split(":=")
        if len(parts) == 2:
            name, value = parts
            overrideArgs[name] = value
        else:
            print "ERROR: invalid syntax for arg %s: %s" % (index, argStr)
            print "       Args must be specified as NAME:=VALUE"
            exit(1)

    ##### Validate the input arguments

    # Make sure the launch file exists
    if not exists(launchFile):
        print "ERROR: Can not find launch file: %s" % launchFile
        exit(2)

    # Make sure the file is actually a launch file (ending on .launch, .test or
    # .xml)
    launchFileBaseName, launchFileExtension = splitext(launchFile.lower())
    if not launchFileExtension in [ ".launch", ".test", ".xml" ] :
        print "ERROR: Must be given a '.launch' file: %s" % launchFile
        exit(3)

    ##### Parse the launch file as XML
    try:
        launchFile = LaunchFile(args, launchFile, overrideArgs=overrideArgs)
    except:
        traceback.print_exc()
        print "ERROR: failed to parse launch file: %s" % launchFile
        exit(4)

    ##### Convert the launch file tree to a dot file
    try:
        graph = launchFile.toDot()
    except:
        traceback.print_exc()
        print "ERROR: failed to generate dot file contents..."
        exit(5)
    else:
        ##### Save the dot file
        graph.write(dotFilename)

        ##### Convert the dot file into a PNG
        if args.convertToPng:
            print "Converting dot file into PNG..."

            # Use the same base name as the dot file for the png
            pngFilename = splitext(dotFilename)[0] + ".png"

            # create png
            graph.draw(pngFilename, prog="dot")
            print "PNG saved to: %s" % pngFilename
