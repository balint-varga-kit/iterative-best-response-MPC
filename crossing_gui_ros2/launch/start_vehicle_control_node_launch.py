"""Launch file to start the pedestrian controller.
You can do the followings:
- Set which config file to use. (parameter set)
- Define the nodes you want to use.
- At the end, decide which nodes to start.
"""

# Launch description generation:
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

# To call and pass argoments to other launch files:
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    ld = LaunchDescription()
    
    # Default config file path:
    default_config_file_name = "params.yaml"
    default_config_file_path = PathJoinSubstitution([FindPackageShare('crossing_gui_ros2'), 'config', default_config_file_name])

    # Arguments:
    #   1: LaunchConfiguration
    #   2: DeclareLaunchArgument
    #   3: add_action
    # (I do not really understand how this works.)
    
    # Define the arguments locally:
    config_file_path = LaunchConfiguration('config_file_path')
    controller_type = LaunchConfiguration('controller_type')
    
    # Allow arguments to be exposed outside of the lauch file:
    config_file_path_launch_arg = DeclareLaunchArgument(
        'config_file_path',
        default_value=default_config_file_path,
        description='Path to the configuration file'
    )
    
    controller_type_lauch_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='control_vehicle_mpc',
        description='Name of the controller python executable (control_vehicle_mpc, control_vehicle_statefeedback).'
    )
    
    # Add the arguments to the launch description: 
    ld.add_action(config_file_path_launch_arg)
    ld.add_action(controller_type_lauch_arg)

    # Log the choosen controller type:
    ld.add_action(LogInfo(msg=["The choosen pedestrian controller type is ",controller_type]))
    
    # Define a node - executable is changing based on argument:
    control_vehicle_node = Node(
        package='crossing_gui_ros2',
        namespace=None,
        executable=controller_type,
        name='control_vehicle_node',
        parameters=[LaunchConfiguration('config_file_path')]
    )

    # Select the nodes you want to start: 
    ld.add_action(control_vehicle_node)

    return ld


