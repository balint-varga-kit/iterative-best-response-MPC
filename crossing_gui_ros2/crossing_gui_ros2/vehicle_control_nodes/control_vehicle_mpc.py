#!/usr/bin/env python3

"""
Car ROS node to publish coordinates for car based on Gui information.
"""

# ============ SETUP: ============

# ================= Imports: =================
# Python:
# ---

# Other files:

# Import class to create node parameters:
from crossing_gui_ros2.simulation_node_functions.create_parameters import DeclareSimulationParameters

# Import ROS2:
import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

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

        Currently we do not need this data.
        """
        pass

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

    def publish_vehicle_input(self) -> None:
        """
        Publishes the vehicle input message.

        We just pass the decision node output to the simulation node, without changing it.
        """
        self.vehicle_input_message.vehicle_x_acceleration = self.decision_node_output.vehicle_acceleration_desired
        self.vehicle_input_message.vehicle_y_acceleration = 0.0
        self.vehicle_input_message.vehicle_displaymessage = self.decision_node_output.ehmi_text_message

        self.publisher_vehicle_input.publish(self.vehicle_input_message)

    def timer_callback(self):
        """
        The node loop, running periodically.
        """

        # Function to publish the calculated state to the gui:
        self.publish_vehicle_input()


def main(args=None):
    # Initialize ROS Client Library for the Python language:
    rclpy.init(args=args)

    executor = SingleThreadedExecutor()

    vehicle_publisher = VehiclePublisher()

    rclpy.spin(vehicle_publisher, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()