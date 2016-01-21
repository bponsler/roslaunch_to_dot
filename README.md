# roslaunch_to_dot
Convert a roslaunch XML file into a dot file containing a graph of the launch tree

This script is capable of handling all of the substitution args (env, optenv, find, anon, arg) and tags (arg, group, include, node test) contained within a launch file. See the [roslaunch XML specification](http://wiki.ros.org/roslaunch/XML) for full details on the XML format of a ROS launch file.

The generated graph will contain the following information:

- one box with a white background for each ROS package that is used by either a launch file, or a node
- one box with a gray background for each ROS XML launch file
- one box with a blue background for each ROS node (labeled by the name of the node)
- one box with a green background for each ROS test node
- one box with a red background for each launch file that is included but cannot be located
- black arrowed lines connecting a launch file to each of the other launch files it includes
- black arrowed lines connecting a launch file to each of the nodes that it launches
- red arrowed lines indicate that a cycle in the launch graph is present (i.e., a launch file includes another launch file that already exists in the launch tree, creating a loop in the tree)

If the script is launched with the **--show-rosparam-nodes** option then the graph will include:

- one circle with a white background for every rosparam file that is included in the launch tree

In the event that a file being included (launch file or rosparam file) uses substitution arguments in its filename to conditionally choose different files, then the line for that connection will be displayed in *orange* and also labeled indicating all of the substitution arguments that were used in the filename.

In the event that two nodes in the launch tree share the same name the second one that is found will be displayed in red and the following warning will be printed to the console:

```
    WARNING: There are two nodes in the launch tree that have the same name: duplicate_name"
```

The "--disable-groups" option is provided the graph will be generated without a subgraph for each package (i.e., the package boxes in the graph will not exist).

If the script is launched with the **--show-node-type** option then each node in the graph representing a ROS node will be labeled using both its name and the ROS package it is contained in. For example, for the ros node "test_node" in the package "my_package" the node would be labeled as follows:

```
    test_node
    type: my_package
```

When the "--show-node-type" option is not provided the node is labeled using only the name of the ROS node it represents.

Additionally, this script provides the ability for the user to override args specified in the provided launch file from the command line. The arguments are provided at the end of the call to the script and use roslaunch argument format. For example, to override the value of the "test_arg" and "index" args, the script could be called as follows:

```
    $ ./roslaunch_to_dot.py example.launch example.dot index:=5 test_arg:=some_value
```

Anywhere in the provided launch file (i.e., example.launch) that the "test_arg" or "index" arguments get used their values will be overriden with "some_value" and "5", respectively. If the launch file passes the arguments to another included launch file, then the overriden values will be passed down to the included launch file as well.

Two command line options have been added to help adjust the layout of the resulting dot graph. Those options are:

    --landscape -- display the graph from left to right, instead of the default which is top to bottom
    --aspect-ratio -- specify the desired aspect ratio of the graph (default is 8.5/11)

## Usage

```
    usage: roslaunch-to-dot.py [-h] [--landscape] [--aspect-ratio ASPECTRATIO]
                               [--png] [--svg] [--pdf]
                               [--disable-groups]
                               [--show-node-type]
                               [--show-rosparam-nodes]
                               launchFile outputFile [arg [arg ...]]

    Create a dot graph file from a ROS launch file.

    positional arguments:
      launchFile            path to the desired launch file
      outputFile            the output dot file to save
      arg                   override an arg specified anywhere in the launch file
                            tree

    optional arguments:
      -h, --help            show this help message and exit
      --landscape           display the nodes from left to right instead of top to
                            bottom
      --aspect-ratio ASPECTRATIO
                            the approximate aspect ratio desired (default =
                            8.5/11)
      --png                 automatically convert the dot file to a PNG
      --svg                 automatically convert the dot file to a SVG
      --pdf                 automatically convert the dot file to a PDF
      --disable-groups      don't group nodes/launch files based on their package
      --show-node-type      label ROS nodes with their type in addition to their
                            name
      --show-rosparam-nodes
                            display nodes and connections for all rosparam files
                            used
```

## Example

Given the following example launch files:

pkg_one/launch/first.launch:

```xml
    <launch>
        <include file="$(find pkg_two)/launch/second.launch" />
        <node pkg="pkg_one" type="node" name="node-1" />
        <node pkg="pkg_two" type="node" name="node-2" />
    </launch>
```

pkg_two/launch/second.launch:

```xml
    <launch>
        <node pkg="pkg_one" type="different_node" name="node-3" />
        <node pkg="pkg_three" type="fun_node" name="node-4" />

        <include file="$(find pkg_three)/launch/third.launch" />
    </launch>
```

and pkg_three/launch/third.launch:

```xml
    <launch>
        <node pkg="pkg_two" type="another_node" name="node-5" />
    </launch>
```

Then running the following command:

```
    $ ./roslaunch-to-dot.py --png ~/sandbox/catkin_ws/src/pkg_one/launch/first.launch examples/first.dot
```

Generates 'examples/first.dot' and 'examples/first.png' which looks like:

![alt text](https://github.com/bponsler/roslaunch_to_dot/raw/master/examples/first.png "Example dot graph")

To see the dot code generated for the example take a look at [examples/first.dot](https://github.com/bponsler/roslaunch_to_dot/raw/master/examples/first.dot).

## Running the unit tests

In order to run the unit tests execute the following commands:

```
    $ cd roslaunch_to_dot
    $ ./run_tests.sh
```

All of the tests should pass without any errors. Make sure to setup the ROS environment in your shell prior to running these scripts, otherwise they will fail.