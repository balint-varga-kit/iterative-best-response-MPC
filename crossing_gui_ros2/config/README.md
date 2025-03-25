

# Useful informations for creating ros yaml parameter configuration files


## Share parameters across multiple nodes:

[Source](https://design.ros2.org/articles/ros_command_line_arguments.html)

Wildcards can be used for node names and namespaces as described in Remapping Names. * matches a single token delimeted by slashes (/). ** matches zero or more tokens delimeted by slashes. Partial matches are not allowed (e.g. foo*).

For example

```YAML
/**:
  ros__parameters:
    string_param: foo
```
will set the parameter string_param on all nodes,
```YAML
/**/some_node:
    ros__parameters:
      string_param: foo
```
will set the parameter string_param on nodes named some_node in any namespace,
```YAML
/foo/*:
    ros__parameters:
      string_param: foo
```
will set the parameter string_param on any node in the namespace /foo.

## Anchors and Aliases in ROS YAML parameter file

Since it would be a nice functionality to set a parameter to specific nodes instead of to all nodes in a namespace, I wanted to use the alias and anchore functionality of the markdown language. The problem is that the ROS YAML parser is not compatibel with it. So I have written a script which takes the config file, parses it with the normal yaml, than deletes all the global variables at the beginning, and saves it. Now I can pass the path to this modified yaml config file to my nodes.
The scripts is in the launch/scripts folder, which is being installed in the same place in the package share directory via the setup.py 