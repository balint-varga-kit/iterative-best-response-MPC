#!/usr/bin/env python3
"""
    Node for controlling the pedestrian object.
    This node publishes the speed information for the pedestrian object in the GUI.
"""

"""
TODOS:

"""

# ================= Imports: =================
# Python:
import sys
import os
import pathlib
import numpy as np

# ROS2:
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor # for declaring parameters

from crossing_gui_ros2.simulation_node_functions.create_parameters import DeclareSimulationParameters
from crossing_gui_ros2.simulation_node_functions.vehicle_class_definition import Vehicle
from crossing_gui_ros2.simulation_node_functions.pedestrian_class_definition import Pedestrian

# ROS2 message and service definitions:
from crossing_gui_ros2_interfaces.msg import PedestrianInputMsg, SimulationOutput

# ================= Constants: =================
# Period of publishing the message:
TIMER_PERIOD = 1/30 # seconds (~ 30 fps)
class PedestrianControlNode(Node):
    """
    The Node for publishing the pedestrian control message.

    Args:

    Attributes:
    """

    def __init__(self):
        super().__init__('pedestrian_control_node')

        # ========== Initialize ROS2 things ==========

        # Declare node paramerers:
        parameter_declarator = DeclareSimulationParameters(self.declare_parameter, self.get_parameter)
        parameter_declarator.declare_parameters_for_pedestrian_control_node()

        # Create timer for publishing:
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        # Create publishers:
        self.publisher_pedestrian_input = self.create_publisher(PedestrianInputMsg, 'simulation/input/pedestrian', 10)

        # Create subscribers:
        self.subscriber_simulation_output = self.create_subscription(SimulationOutput, 'simulation/output', self.callback_simulation_output, 10)

        # 
        self.pedestrian = Pedestrian(initial_x_position=self.get_parameter('pedestrian_initial_x').value,
                                     initial_y_position=self.get_parameter('pedestrian_initial_y').value)
        self.vehicle = Vehicle()

        self.pedestrian_input_message = PedestrianInputMsg()

        # System state:
        self.systemstate = np.array([0.0, 0.0, 0.0, 0.0])

        transformed_state_feedback_matrix = np.array([0.0, 0.0, 3.8053, -1.0000])

        # Desired velocities:
        xd_s = 8 # m/s - ca. 30 kmh
        yd_s = - 1.5 # m/s - ca. 6kmh

        # Pedestrian Crossing position:
        x_ped = 60.0 # m
        y_ped = 15.0 # m

        # x_transformed = T1 * x

        T_inv = np.array([[1-xd_s,       0,      0,       0],
                           [     0, 1-x_ped,      0,       0],
                           [     0,       0, 1-yd_s,       0],
                           [     0,       0,      0, 1-y_ped]])
        
        T = np.linalg.inv(T_inv)

        self.state_feedback_matrix = transformed_state_feedback_matrix @ T_inv

        # END OF INIT

    def callback_simulation_output(self, msg: SimulationOutput) -> None:
        """Callback function for the 'simulation/output' subscriber.
        """

        self.pedestrian.x_position     = msg.pedestrian_x
        self.pedestrian.y_position     = msg.pedestrian_y
        self.pedestrian.x_speed        = msg.pedestrian_x_speed
        self.pedestrian.y_speed        = msg.pedestrian_y_speed

        self.vehicle.x_position     = msg.vehicle_x
        self.vehicle.y_position     = msg.vehicle_y
        self.vehicle.x_speed        = msg.vehicle_x_speed
        self.vehicle.y_speed        = msg.vehicle_y_speed

    def calculate_pedestrian_input(self) -> np.float64:

        systemstate = np.array([self.vehicle.x_speed,
                                     self.vehicle.x_position,
                                     self.pedestrian.y_speed,
                                     self.pedestrian.y_position])

        pedestrian_input = - np.transpose(self.state_feedback_matrix) @ systemstate
        return pedestrian_input
    
    def publish_pedestrian_input(self, pedestrian_input: np.float64) -> None:

        self.pedestrian_input_message.pedestrian_x_acceleration = 0.0
        self.pedestrian_input_message.pedestrian_y_acceleration = pedestrian_input
        self.pedestrian_input_message.pedestrian_intention      = 0.0
        self.pedestrian_input_message.pedestrian_displaymessage = "pedestrian control node init"

        # Publish the message into 'Move_Pedestrian_with_arrows' topic:
        self.publisher_pedestrian_input.publish(self.pedestrian_input_message)

    def timer_callback(self):
        """
        """

        pedestrian_input = self.calculate_pedestrian_input()
        self.publish_pedestrian_input(pedestrian_input)


def main(args=None):
    rclpy.init(args=args)
    pedestrian_control_node = PedestrianControlNode()
    rclpy.spin(pedestrian_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pedestrian_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()