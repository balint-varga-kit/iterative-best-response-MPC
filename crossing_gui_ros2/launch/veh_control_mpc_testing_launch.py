"""Launch file for state feedback control for both vehicle and pedestrian.
You can do the followings:
- Set which config file to use. (parameter set)
- Define the nodes you want to use.
- At the end, decide which nodes to start.

It is not necessary to rebuild if you modify! nice :D
"""

# test comment

# Import YAML config files:
import os
import yaml
from ament_index_python.packages import get_package_share_directory

import sys
sys.path.append(os.path.join(os.path.dirname(__file__), 'scripts'))
from anchor_and_alias_yaml_support import anchor_and_alias_yaml_support
# The problem is that if i modify the line above like "scripts.anchor_and_alias_yaml_support..." pylance works,
# but upon launching the launch file it gves the error "No module named ..." No Idea, I just leave it like this.

# Launch description generation:
from launch import LaunchDescription
from launch_ros.actions import Node

# To call and pass argoments to other launch files:
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():

    ld = LaunchDescription()

    # Choose the parameter set:
    config_file_name = "params.yaml"
    corrected_config_file_name = "corrected_params.yaml"

    config_folder_path = os.path.join(get_package_share_directory('crossing_gui_ros2'), 'config')
    config_file_path = os.path.join(config_folder_path, config_file_name)
    corrected_config_file_path = os.path.join(config_folder_path, corrected_config_file_name)

    anchor_and_alias_yaml_support(input_file_path=config_file_path, 
                                  output_file_path=corrected_config_file_path)

    # (Wrong argument value not only gives an error, but starts a background process with high cpu usage, which you have to kill manually.)
    control_vehicle_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('crossing_gui_ros2'), 'launch', 'start_vehicle_control_node_launch.py'
                ])
            ]),
        launch_arguments = {
            "config_file_path": corrected_config_file_path,
            "controller_type": "control_vehicle_mpc" # control_vehicle_mpc, control_vehicle_statefeedback
        }.items()
    )

    control_pedestrian_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('crossing_gui_ros2'), 'launch', 'start_pedestrian_control_node_launch.py'
                ])
            ]),
        launch_arguments = {
            "config_file_path": corrected_config_file_path,
            "controller_type": "control_pedestrian_keyboard" # control_pedestrian_keyboard, control_pedestrian_statefeedback
        }.items()
    )


    simulation_node = Node(
        package='crossing_gui_ros2',
        namespace=None,
        executable='simulation_node',
        name='simulation_node',
        parameters=[corrected_config_file_path]
    )

    simulation_gui_node = Node(
        package='crossing_gui_ros2',
        namespace=None,
        executable='simulation_gui_node',
        name='simulation_gui_node',
        parameters=[corrected_config_file_path]
    )

    decision_making_node = Node(
        package='decision_making_package',
        namespace=None,
        executable='decision_making_node', # decision_making_node, decision_making_node_rulebased
        name='decision_making_node',
        parameters=[corrected_config_file_path]
    )

    log_node = Node(
        package='crossing_gui_ros2',
        namespace=None,
        executable='log',
        name='log_node',
        parameters=[corrected_config_file_path]
    )

    #  Select the nodes you want to start: 
    ld.add_action(control_vehicle_node)
    ld.add_action(control_pedestrian_node)
    
    ld.add_action(simulation_node)
    ld.add_action(simulation_gui_node)
    
    ld.add_action(decision_making_node)

    ld.add_action(log_node)

    return ld
