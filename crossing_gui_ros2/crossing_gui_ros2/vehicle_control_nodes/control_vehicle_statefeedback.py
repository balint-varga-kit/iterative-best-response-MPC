#!/usr/bin/env python3

"""
Car ROS node to publish coordinates for car based on Gui information.
"""

# ============ SETUP: ============

# ================= Imports: =================
# Python:
import numpy as np
from typing import Type
from dataclasses import dataclass

# Other files:

# Import the file "create_parameters.py" which contains the "DeclareSimulationParameters" class:
# (Build the package first, otherwise it won't work, I have no idea why.)
from crossing_gui_ros2.simulation_node_functions.create_parameters import DeclareSimulationParameters
from crossing_gui_ros2.simulation_node_functions.vehicle_class_definition import Vehicle
from crossing_gui_ros2.simulation_node_functions.pedestrian_class_definition import Pedestrian

# Import ROS2:
import rclpy
from rclpy.node import Node

# Import ROS2 message definitions:
from crossing_gui_ros2_interfaces.msg import VehicleInputMsg, SimulationOutput
from decision_making_interface.msg import EhmiDecisionResult

# ================= Constants: =================
# Define publishing rate:
TIMER_PERIOD = 0.01  # seconds (manually)

class VehiclePublisher(Node):

    def __init__(self):
        super().__init__('control_vehicle_node')

        # Allocate vehicle control message:
        self.vehicle_input_message = VehicleInputMsg()

        # Allocate decision node output:
        self.decision_node_output = EhmiDecisionResult()

        # ========== Initialize ROS2 things: ==========

        # Declare node paramerers:
        parameter_declarator = DeclareSimulationParameters(self.declare_parameter, self.get_parameter)
        parameter_declarator.declare_parameters_for_vehicle_control_node()

        # Initialize the vehicle and pedestrian objects:
        self.vehicle = Vehicle(
            initial_x_position     = self.get_parameter('vehicle_initial_x').value,
            initial_y_position     = self.get_parameter('vehicle_initial_y').value,
            initial_x_speed        = self.get_parameter('vehicle_initial_x_speed').value,
            initial_y_speed        = self.get_parameter('vehicle_initial_y_speed').value,
            initial_displaymessage = "vehicle control node init")
        self.pedestrian = Pedestrian()

        # Subscribers:
        self.subscriber_simulation_feedback = self.create_subscription(SimulationOutput, 'simulation/output', self.callback_simulation_feedback, 10)
        self.subscriber_decision_node       = self.create_subscription(EhmiDecisionResult, 'negotiation_node/out/decision_result', self.decision_node_callback, 10)

        # Publishers:
        self.publisher_vehicle_input = self.create_publisher(VehicleInputMsg, 'simulation/input/vehicle', 10)

        # Publish the data with given frequency:
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def callback_simulation_feedback(self, msg: SimulationOutput) -> None:
        """
        Saves the output of the simulation node locally for further usage.
        """

        self.pedestrian.x_position     = msg.pedestrian_x
        self.pedestrian.y_position     = msg.pedestrian_y
        self.pedestrian.x_speed        = msg.pedestrian_x_speed
        self.pedestrian.y_speed        = msg.pedestrian_y_speed
        self.pedestrian.displaymessage = msg.pedestrian_displaymessage
        self.pedestrian.intention      = msg.pedestrian_intention

        self.vehicle.x_position     = msg.vehicle_x
        self.vehicle.y_position     = msg.vehicle_y
        self.vehicle.x_speed        = msg.vehicle_x_speed
        self.vehicle.y_speed        = msg.vehicle_y_speed
        self.vehicle.displaymessage = msg.vehicle_displaymessage

    def decision_node_callback(self, arrived_data: EhmiDecisionResult) -> None:
        """
        Saves the output of the decision node locally for further usage.

        Args:
            arrived_data (EhmiDecisionResult): Output of the decision node
        
        Help:
            EhmiDecisionResult.msg:
                float64 vehicle_speed_desired 
                float64 vehicle_acceleration_desired
                string ehmi_text_message
                int64 message_index
        """

        self.decision_node_output = arrived_data

    def compute_vehicle_input(self) -> np.float64:
        """
        Calculate the input of the vehicle from current and desired state.

        Returns:
            np.float64: Calculated input for the vehicle.
        """

        # State feedback calculated from optimization script:
        K_vehicle = np.array([1.0, 0.0, 0.0, 0.0])

        # Get current state:
        state_vector = np.array([self.vehicle.x_speed,
                                 self.vehicle.x_position,
                                 self.pedestrian.y_speed,
                                 self.pedestrian.y_position])

        # Goal state:
        x_setpoint = np.array([8.0, 90.0, -1.5, 0.0])

        # Error:
        x_error = state_vector - x_setpoint

        vehicle_input = - np.transpose(K_vehicle) @ x_error

        return vehicle_input
    
    def publish_vehicle_input(self, vehicle_input: np.float64) -> None:
        """
        Publishes the vehicle input message.

        Args:
            vehicle_input (np.float64): The vehicle input value.
        """
        self.vehicle_input_message.vehicle_x_acceleration = vehicle_input
        self.vehicle_input_message.vehicle_y_acceleration = 0.0
        self.vehicle_input_message.vehicle_displaymessage = self.vehicle.displaymessage

        self.publisher_vehicle_input.publish(self.vehicle_input_message)

    def timer_callback(self):
        """The node loop, running periodically.

        The first half is about the service request for initial information,
        the second half is about publishing the new positions for the vehicle.
        """

        # Controller function which calculates the input of the vehicle:
        vehicle_input = self.compute_vehicle_input()

        # Function to publish the calculated state to the gui:
        self.publish_vehicle_input(vehicle_input)

def main(args=None):
    # Initialize ROS Client Library for the Python language:
    rclpy.init(args=args)

    vehicle_publisher = VehiclePublisher()

    rclpy.spin(vehicle_publisher, executor=None)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()