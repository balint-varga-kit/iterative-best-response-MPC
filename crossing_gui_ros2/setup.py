from setuptools import setup
from glob import glob
import os


package_name = 'crossing_gui_ros2'

simulation_node_functions_folder_name = 'simulation_node_functions'
pedestrian_control_nodes_folder_name = 'pedestrian_control_nodes'
vehicle_control_nodes_folder_name = 'vehicle_control_nodes'


path_to_simulation_node_functions = os.path.join(package_name, simulation_node_functions_folder_name)
path_to_pedestrian_control_nodes = os.path.join(package_name, pedestrian_control_nodes_folder_name)
path_to_vehicle_control_nodes = os.path.join(package_name, vehicle_control_nodes_folder_name)
path_to_launch_file_python_scripts = os.path.join('launch', 'scripts')

setup(
    name=package_name,
    version='0.0.0',
    # The next line copies the python packages to the install dir, e.g. the decision_functions:
    packages=[package_name,
              path_to_simulation_node_functions,
              path_to_pedestrian_control_nodes,
              path_to_vehicle_control_nodes,
              path_to_launch_file_python_scripts],

    package_data={
      'crossing_gui_ros2': ['fonts/*.ttf'],
    },
    include_package_data=True,
    
    data_files=[
        # List of tuples where the first element is the path to the directory where the files should be installed,
        # and the second element is a list of files to install there.

        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install the launch files and launch file scripts:
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch', 'scripts'), glob(os.path.join('launch', 'scripts', '*.[pxy][yma]*'))),

        # Install the YAML config files:
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Additional resources:
        (os.path.join('lib', 'python3.8', 'site-packages', package_name, 'imgs'), glob('crossing_gui_ros2/imgs/*.png') ),
        (os.path.join('lib', 'python3.8', 'site-packages', package_name, 'fonts'), glob('crossing_gui_ros2/fonts/*.ttf') ),
        #

        # Syntax:  ( target_directory, [list of files to be put there] )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    # maintainer, email, description and license should be the same as in package.xml
    # Reason:
    # The files have different purposes, package.xml is for ROS-specific tools, setup.py is for Python package management.
    maintainer='bogdan',
    maintainer_email='urxel@student.kit.edu',
    description='Simulation of the pedestrian crossing scenario in urxel master thesis.',
    license='Apache 2.0',
    tests_require=['pytest'], # this is necessary for test codes
    entry_points={
        'console_scripts': [
            # Vehile control nodes:
            'control_vehicle_mpc = crossing_gui_ros2.vehicle_control_nodes.control_vehicle_mpc:main',

            # Pedestrian control nodes:
            'control_pedestrian_keyboard = crossing_gui_ros2.pedestrian_control_nodes.control_pedestrian_keyboard:main',

            # Simulation nodes:
            'simulation_node = crossing_gui_ros2.simulation_node:main',
            'simulation_gui_node = crossing_gui_ros2.simulation_gui_node:main',
            
            # Other nodes:
            'log = crossing_gui_ros2.log:main',

        # For example:
        # test = my_python_pkg.my_python_node:main
        # 'test' is the name of the executable after the script is installed
        # 'my_python_pkg.my_python_node:main' means execute the main() function
        # inside of the 'my_python_node.py' file, the node starts here.
        ],
    },
)
