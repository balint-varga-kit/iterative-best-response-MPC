#!/usr/bin/env python

"""
This script defines the SimulationNode class, which is a ROS2 node responsible for simulating the movement of a vehicle and a pedestrian.
The node receives input messages for the vehicle and pedestrian, updates their states based on the received inputs, and publishes the simulation output.
"""

# ================= Imports: =================
import numpy as np
from dataclasses import dataclass

# Import the file "create_parameters.py" which contains the "DeclareSimulationParameters" class:
# (Build the package first, otherwise it won't work, I have no idea why.)
from crossing_gui_ros2.simulation_node_functions.create_parameters import DeclareSimulationParameters
from crossing_gui_ros2.simulation_node_functions.vehicle_class_definition import Vehicle
from crossing_gui_ros2.simulation_node_functions.pedestrian_class_definition import Pedestrian

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from std_msgs.msg import Float64

from crossing_gui_ros2_interfaces.msg import VehicleInputMsg, PedestrianInputMsg, PedestrianSpeedInputMsg, SimulationOutput
from decision_making_interface.msg import PedestrianIntentionStateList, PedestrianIntentionState

class SimulationNode(Node):
    def __init__(self):
        """
        Initializes the SimulationNode class by creating publishers, subscribers, and declaring node parameters.
        """
        super().__init__('simulation_node') 

        self.simulation_output_message = SimulationOutput()

        # Subscribers:
        self.vehicle_input_subscriber = self.create_subscription(VehicleInputMsg, 'simulation/input/vehicle', self.vehicle_input_callback, 10)
        self.pedestrian_input_subscriber = self.create_subscription(PedestrianInputMsg, 'simulation/input/pedestrian', self.pedestrian_input_callback, 10)
        self.pedestrian_speed_input_subscriber = self.create_subscription(PedestrianSpeedInputMsg, 'simulation/input/pedestrian_speed', self.pedestrian_speed_input_callback, 10)
        self.pedestrian_control_type = "Not set yet."
        self.pedestrian_intention_decay_value_subscriber = self.create_subscription(Float64, 'negotiation_node/out/intention_decay_value', self.pedestrian_intention_decay_value_callback, 10)
        self.pedestrian_yspeed_decay_value_subscriber = self.create_subscription(Float64, 'negotiation_node/out/pedyspeed_decay_value', self.pedestrian_yspeed_decay_value_callback, 10)

        # Publishers:
        self.simulation_output_publisher = self.create_publisher(SimulationOutput, 'simulation/output', 10)
        self.pedestrian_states_publisher = self.create_publisher(PedestrianIntentionStateList, 'relevant_pedestrian/intention_position', 10)

        # Services:
        self.reset_vehicle_service = self.create_service(Empty, '/reset_vehicle', self.reset_vehicle_callback)

        # Declare node paramerers:
        parameter_declarator = DeclareSimulationParameters(self.declare_parameter, self.get_parameter)
        parameter_declarator.declare_parameters_for_simulation_node()

        # Initialize vehicle and pedestrian states:
        self.vehicle = Vehicle(initial_x_position=self.get_parameter('vehicle_initial_x').value,
                               initial_y_position=self.get_parameter('vehicle_initial_y').value,
                               initial_x_speed=self.get_parameter('vehicle_initial_x_speed').value,
                               initial_y_speed=self.get_parameter('vehicle_initial_y_speed').value)
        self.pedestrian = Pedestrian(initial_x_position=self.get_parameter('pedestrian_initial_x').value,
                                     initial_y_position=self.get_parameter('pedestrian_initial_y').value)

        # State update frequency:
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def reset_vehicle_callback(self, request: Empty, response: Empty) -> Empty:
        """
        Reset the vehicle's position and speed to their initial values.

        Args:
            request: The request object (not used in this method).
            response: The response object (not used in this method).

        Returns:
            None
        """
        # self.vehicle.x_position = self.get_parameter('vehicle_initial_x').value
        # self.vehicle.y_position = self.get_parameter('vehicle_initial_y').value
        # self.vehicle.x_speed = self.get_parameter('vehicle_initial_x_speed').value
        # self.vehicle.y_speed = self.get_parameter('vehicle_initial_y_speed').value

        self.vehicle.reset_to_initial_state()

        self.pedestrian.reset_to_initial_state()


        return response

    def timer_callback(self):
        """
        This method is called periodically by the timer.
        It updates the vehicle and pedestrian states and publishes the simulation output.
        """
        self.update_vehicle()
        self.update_pedestrian()
        self.publish_simulation_output()
        self.publish_pedestrian_intention_state_list_msg()

    def vehicle_input_callback(self, msg: VehicleInputMsg) -> None:
        """
        Subscriber to the vehicle input and save it locally.

        Args:
            msg (VehicleInputMsg): An instance of the VehicleInputMsg message type, containing information about the vehicle's acceleration and display message.
                float64 vehicle_x_acceleration
                float64 vehicle_y_acceleration
                string vehicle_displaymessage
        """
        self.vehicle.x_acceleration = msg.vehicle_x_acceleration
        self.vehicle.y_acceleration = msg.vehicle_y_acceleration
        self.vehicle.displaymessage = msg.vehicle_displaymessage

    def pedestrian_input_callback(self, msg: PedestrianInputMsg) -> None:
        """
        Subscriber to the pedestrian input and save it locally.

        Args:
            msg (PedestrianInputMsg): Acceleration type input for the pedestrian.
                float64 pedestrian_x_acceleration
                float64 pedestrian_y_acceleration
                float64 pedestrian_intention
                string pedestrian_displaymessage
        """
        control_type = "Acceleration"  # This callback is for acceleration type control.

        if self.pedestrian_control_type == "Not set yet.":
            # If the control type has not been set yet, set it:
            self.pedestrian_control_type = control_type
        elif self.pedestrian_control_type != control_type:
            # If the control type has already been set to a different type, raise an exception
            self.get_logger().error(f"Multiple control inputs detected! Current: {self.pedestrian_control_type}, Incoming: {control_type}")
            raise Exception(f"Multiple control inputs detected! Current: {self.pedestrian_control_type}, Incoming: {control_type}")

        self.pedestrian.x_acceleration   = msg.pedestrian_x_acceleration
        self.pedestrian.y_acceleration   = msg.pedestrian_y_acceleration
        self.pedestrian.intention        = msg.pedestrian_intention
        self.pedestrian.displaymessage   = msg.pedestrian_displaymessage

    def pedestrian_speed_input_callback(self, msg: PedestrianSpeedInputMsg) -> None:
        """
        Subscriber to the pedestrian input and save it locally.

        Args:
            msg (PedestrianSpeedInputMsg): Speed type input for the pedestrian.
                float64 pedestrian_x_speed
                float64 pedestrian_y_speed
                float64 pedestrian_intention
                string pedestrian_displaymessage
        """

        control_type = "Speed"  # This callback is for acceleration type control.

        if self.pedestrian_control_type == "Not set yet.":
            # If the control type has not been set yet, set it:
            self.pedestrian_control_type = control_type
        elif self.pedestrian_control_type != control_type:
            # If the control type has already been set to a different type, raise an exception
            self.get_logger().error(f"Multiple control inputs detected! Current: {self.pedestrian_control_type}, Incoming: {control_type}")
            raise Exception(f"Multiple control inputs detected! Current: {self.pedestrian_control_type}, Incoming: {control_type}")

        self.pedestrian.x_speed   = msg.pedestrian_x_speed
        self.pedestrian.y_speed   = msg.pedestrian_y_speed
        self.pedestrian.intention        = msg.pedestrian_intention
        self.pedestrian.displaymessage   = msg.pedestrian_displaymessage
        
    def pedestrian_intention_decay_value_callback(self, msg: Float64) -> None:
        """
        Subscriber to the pedestrian intention decay value and save it locally.

        Args:
            msg (Float64): The decay value for the pedestrian intention.
        """
        self.pedestrian.intention_decay = msg.data
        
    def pedestrian_yspeed_decay_value_callback(self, msg: Float64) -> None:
        """
        Subscriber to the pedestrian y speed decay value and save it locally.

        Args:
            msg (Float64): The decay value for the pedestrian y speed.
        """
        self.pedestrian.y_speed_smooth_decay = msg.data

    def update_vehicle(self) -> None:
        """
        Updates the vehicle's state by integrating acceleration to update speed,
        enforcing speed constraints, and integrating speed to update position.
        Speed constraints are enforced to set negative speed to zero and to set speed to zero
        if it falls below a numerical threshold.
        """

        # Update speed (1. integration):
        self.vehicle.x_speed += self.vehicle.x_acceleration * self.timer_period
        self.vehicle.y_speed += self.vehicle.y_acceleration * self.timer_period

        # Enforce some speed constraints:
        # The vehicle can't drive in reverse.
        if self.vehicle.x_speed < 0.0: self.vehicle.x_speed = 0.0
        if self.vehicle.y_speed < 0.0: self.vehicle.y_speed = 0.0

        # If the speed is small enough, just change it to zero.
        numerical_speed_threshold = 0.001
        if abs(self.vehicle.x_speed) < numerical_speed_threshold: self.vehicle.x_speed = 0.0
        if abs(self.vehicle.y_speed) < numerical_speed_threshold: self.vehicle.y_speed = 0.0

        # Update position (2. integration):
        self.vehicle.x_position += self.vehicle.x_speed * self.timer_period
        self.vehicle.y_position += self.vehicle.y_speed * self.timer_period

    def update_pedestrian(self) -> None:
        """
        Updates the pedestrian's state by integrating acceleration to update speed,
        enforcing speed constraints, and integrating speed to update position.
        Speed constraints are enforced to set speed to zero if it falls below a numerical threshold.
        """

        # Update speed (1. integration):
        # (Only if pedestrian is acceleration controlled)
        if self.pedestrian_control_type == "Acceleration":
            self.pedestrian.x_speed += self.pedestrian.x_acceleration * self.timer_period
            self.pedestrian.y_speed += self.pedestrian.y_acceleration * self.timer_period

        # Enforce some speed constraints:
        # If the speed is small enough, just change it to zero.
        numerical_speed_threshold = 0.001
        if abs(self.pedestrian.x_speed) < numerical_speed_threshold: self.pedestrian.x_speed = 0.0
        if abs(self.pedestrian.y_speed) < numerical_speed_threshold: self.pedestrian.y_speed = 0.0

        # Update position (2. integration):
        self.pedestrian.x_position += self.pedestrian.x_speed * self.timer_period
        self.pedestrian.y_position += self.pedestrian.y_speed * self.timer_period

    def publish_simulation_output(self) -> None:
        """
        Publishes simulation output by updating the simulation output message attributes
        with current system state information for both pedestrian and vehicle, 
        and then publishes the updated message using the simulation output publisher.
        """
        self.simulation_output_message.pedestrian_x                 = self.pedestrian.x_position
        self.simulation_output_message.pedestrian_y                 = self.pedestrian.y_position
        self.simulation_output_message.pedestrian_x_speed           = self.pedestrian.x_speed
        self.simulation_output_message.pedestrian_y_speed           = self.pedestrian.y_speed
        self.simulation_output_message.pedestrian_x_acceleration    = self.pedestrian.x_acceleration
        self.simulation_output_message.pedestrian_y_acceleration    = self.pedestrian.y_acceleration
        self.simulation_output_message.pedestrian_displaymessage    = self.pedestrian.displaymessage
        self.simulation_output_message.pedestrian_intention         = self.pedestrian.intention
        self.simulation_output_message.pedestrian_intention_decay   = self.pedestrian.intention_decay
        self.simulation_output_message.pedestrian_y_speed_smooth_decay = self.pedestrian.y_speed_smooth_decay
        self.simulation_output_message.vehicle_x                    = self.vehicle.x_position
        self.simulation_output_message.vehicle_y                    = self.vehicle.y_position
        self.simulation_output_message.vehicle_x_speed              = self.vehicle.x_speed
        self.simulation_output_message.vehicle_y_speed              = self.vehicle.y_speed
        self.simulation_output_message.vehicle_x_acceleration       = self.vehicle.x_acceleration
        self.simulation_output_message.vehicle_y_acceleration       = self.vehicle.y_acceleration
        self.simulation_output_message.vehicle_displaymessage       = self.vehicle.displaymessage

        self.simulation_output_publisher.publish(self.simulation_output_message)

    def publish_pedestrian_intention_state_list_msg(self) -> None:
            """
            Publishes a list of states and intentions of pedestrian crossing and pedestrian.

            This function retrieves retrieves the position of the pedestrian crossing and converts it
            to the vehicle coordinate system. Than it retreives the position and speed of the pedestrian
            and converts it to the vehicle coordinate system as well. Finally it creates the
            PedestrianIntentionStateList message, populates it with the converted data and publishes it.
            The first item in the list is the pedestrian crossing data, the second item is the pedestrian
            data.

            Args:
                None

            Returns:
                None
            """ 

            # Main message:
            decision_making_node_msg = PedestrianIntentionStateList()
            decision_making_node_msg.header.stamp = self.get_clock().now().to_msg()
            decision_making_node_msg.header.frame_id = "in vehicle coordinate system"


            # Get pedestrian crossing position relative to the vehicle in GUI coordinate system:
            r_veh_0 = np.array([self.vehicle.x_position, self.vehicle.y_position, 0.0])
            r_pedcros_0 = np.array([self.get_parameter("pedestrian_crossing_x").value, self.get_parameter("road_y").value, 0.0])
            r_pedcros_1 = r_pedcros_0 - r_veh_0

            # Get pedestrian position velocity relative to the vehicle in GUI coordinate system:
            # Rule: v_P20 = v_P21 + V_10 + w_10 x rho_P21 # see Dinamika jegyzet - relativ kinematika p93
            v_pedcros_0 = np.array([0.0, 0.0, 0.0])
            v_veh_0 = np.array([self.vehicle.x_speed, self.vehicle.y_speed, 0.0])
            w_veh_0 = np.array([0.0, 0.0, 0.0])

            v_pedcros_1 = v_pedcros_0 - v_veh_0 - np.cross(w_veh_0, r_pedcros_1)

            # Transformation matrix from vehicle sys to gui sys:
            # (vehicle coord sys basis vectors expressed in gui coord sys)
            A_vehicle_to_gui = np.array([[1.0,  0.0],
                                         [0.0, -1.0]])
            # A_gui_to_vehicle = np.linalg.inv(A_vehicle_to_gui)
            A_gui_to_vehicle = A_vehicle_to_gui # inverse is the transpose which is the same in this case

            # Convert the vectors into vehicle coordinate system:
            r_pedcros_1_vehicle = A_gui_to_vehicle @ r_pedcros_1[:2]
            v_pedcros_1_vehicle = A_gui_to_vehicle @ v_pedcros_1[:2]


            # Get pedestrian position and speed relative to the vehicle in GUI coordinate system:
            r_ped_0 = np.array([self.pedestrian.x_position, self.pedestrian.y_position, 0.0])
            r_ped_1 = r_ped_0 - r_veh_0

            v_ped_0 = np.array([self.pedestrian.x_speed, self.pedestrian.y_speed, 0.0])

            v_ped_1 = v_ped_0 - v_veh_0 - np.cross(w_veh_0, r_ped_1)

            # Convert the vectors into vehicle coordinate system:
            r_ped_1_vehicle = A_gui_to_vehicle @ r_ped_1[:2]
            v_ped_1_vehicle = A_gui_to_vehicle @ v_ped_1[:2]

            # Pedestrian crossing in vehicle coordinate system:
            pedestrian_0_intention_and_state = PedestrianIntentionState()
            pedestrian_0_intention_and_state.header.stamp= self.get_clock().now().to_msg()
            pedestrian_0_intention_and_state.header.frame_id = "pedcros relative to vehicle in vehicle coord system"
            pedestrian_0_intention_and_state.id = "0"
            pedestrian_0_intention_and_state.intention = 0.0 # (Does not have one :D)
            pedestrian_0_intention_and_state.position = r_pedcros_1_vehicle
            pedestrian_0_intention_and_state.velocity = v_pedcros_1_vehicle

            # Pedestrian in vehicle coordinate system:
            pedestrian_1_intention_and_state = PedestrianIntentionState()
            pedestrian_1_intention_and_state.header.stamp= self.get_clock().now().to_msg()
            pedestrian_1_intention_and_state.header.frame_id = "vehicle coordinate system"
            pedestrian_1_intention_and_state.id = "0"
            pedestrian_1_intention_and_state.intention = self.pedestrian.intention
            pedestrian_1_intention_and_state.position = r_ped_1_vehicle
            pedestrian_1_intention_and_state.velocity = v_ped_1_vehicle

            decision_making_node_msg.pedestrian_states = []
            decision_making_node_msg.pedestrian_states.append(pedestrian_0_intention_and_state)
            decision_making_node_msg.pedestrian_states.append(pedestrian_1_intention_and_state)

            self.pedestrian_states_publisher.publish(decision_making_node_msg)

def main(args=None):
    rclpy.init(args=args)

    simulation_node = SimulationNode()

    rclpy.spin(simulation_node, executor=None)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()