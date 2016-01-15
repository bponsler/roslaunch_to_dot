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

## Usage

```bash
	$ ./roslaunch-to-dot.py --help
    usage: roslaunch-to-dot.py [-h] [--png] launchFile outputFile

    Create a dot graph file from a ROS launch file.

    positional arguments:
      launchFile  path to the desired launch file
      outputFile  the output dot file to save

    optional arguments:
      -h, --help  show this help message and exit
      --png       automatically convert the dot file to a PNG
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

```bash
    $ ./roslaunch-to-dot.py --png ~/sandbox/catkin_ws/src/pkg_one/launch/first.launch examples/first.dot
```

Generates 'examples/first.dot' and 'examples/first.png' which looks like:

![alt text](https://github.com/bponsler/roslaunch_to_dot/raw/master/examples/first.png "Example dot graph")

To see the dot code generated for the example take a look at [examples/first.dot](https://github.com/bponsler/roslaunch_to_dot/raw/master/examples/first.dot).