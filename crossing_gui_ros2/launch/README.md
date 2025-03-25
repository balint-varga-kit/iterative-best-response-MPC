# Infos for the launch files:

## Loading .yaml config file:

The problem was that I wanted to load the same initial state of the system into multiple nodes. Since in ROS2 parameters are unique to the nodes, I needed a workaround for "global" parameters. 

I found out about anchors and aliases in YAML files so that I can create a shared value across all nodes, and I only have to set the value once.

The Problem was that this functionality of the YAML syntax is not yet supported by the ROS2 YAML parser.

Workaround:

- https://answers.ros.org/question/346409/ros2-component_container-yaml-parsing/
- https://github.com/osrf/ros2_serial_example/blob/master/ros2_serial_example/launch/px4.launch.py

So instead of this:
~~~
config = os.path.join(get_package_share_directory('crossing_gui_ros2'), 'config', config_file_name)
~~~
Use this:
~~~
with open(param_config, 'r') as f:
        params = yaml.safe_load(f)['iris_0/']['ros__parameters']
~~~

This means we load and parse within the launch file instead of within the node.

Just leave out the brackets to load the whole yaml file.

## Naming:

The node names in the launch files and the node names in the .yaml parameter file must be the same to load the parameters correctly in the appropriate node.