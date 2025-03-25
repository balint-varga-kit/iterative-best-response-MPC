#!/usr/bin/env python

# Import Python:
import numpy as np
from typing import List

import rclpy
from rclpy.node import Node
from decision_making_interface.msg import (
    # PedestrianIntentionState,
    EhmiDecisionResult,
    PedestrianIntentionStateList,
)

from std_msgs.msg import Float64

from crossing_gui_ros2_interfaces.msg import SystemStatesMsg, SystemStateMsg
from crossing_gui_ros2_interfaces.srv import GetGuiParam

from .vehicle_class_definition import Vehicle
from .pedestrian_class_definition import Pedestrian

from .utils.simple_cruise_controller import SimpleCruiseController
from .utils.optimization_based_controller import OptimizationBasedController

# Constants:
TIMER_PERIOD = 0.2
PREDICTION_HORIZON = 10

np.set_printoptions(precision=4, suppress=True, linewidth=np.inf)


class DecisionMakingNode(Node):
    def __init__(self):
        super().__init__("decision_making_node")

        # Creating Subscribers and Publishers: #
        self.subscriber_pedestrian_intention_state_list = self.create_subscription(
            PedestrianIntentionStateList,
            "relevant_pedestrian/intention_position",
            self.pedestrian_intention_state_list_callback,
            10,
        )

        self.publisher_decision_result = self.create_publisher(
            EhmiDecisionResult, "negotiation_node/out/decision_result", 10
        )
        self.publisher_vehicle_state_prediction = self.create_publisher(
            SystemStatesMsg, "negotiation_node/out/vehicle_state_prediction", 10
        )
        self.publisher_intention_decay_value = self.create_publisher(
            Float64, "negotiation_node/out/intention_decay_value", 10
        )
        self.publisher_pedyspeed_decay_value = self.create_publisher(
            Float64, "negotiation_node/out/pedyspeed_decay_value", 10
        )

        self.get_gui_param_client = self.create_client(
            GetGuiParam, "/gui/get_gui_param"
        )
        while not self.get_gui_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "/gui/get_gui_param service not available, waiting..."
            )

        # Initialize the decision result message:
        self.ehmi_decision_result_message = EhmiDecisionResult()
        self.ehmi_decision_result_message.vehicle_speed_desired = 0.0  # float64
        self.ehmi_decision_result_message.vehicle_acceleration_desired = 0.0  # float64
        self.ehmi_decision_result_message.ehmi_text_message = (
            "Decision node init"  # string
        )
        self.ehmi_decision_result_message.message_index = 0  # int64

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        # Initialize the vehicle and pedestrian objects:
        self.vehicle = Vehicle(5.0, 0.0, 0.0, 10.0, 0.0, 0.0, "decision node init")
        self.pedestrian = Pedestrian(
            40.0, 0.0, 0.0, 15.0, 0.0, 0.0, 0.0, "decision node init", TIMER_PERIOD
        )
        self.vehicle.target_state = np.array(
            [0.0, 8.0, 0.0, 0.0]
        )  # average driving speed
        self.pedestrian.target_state = np.array(
            [0.0, 0.0, 0.0, 1.4]
        )  # average walking speed

        self.is_there_a_pedestrian = False

        self.optimization_type = "joint_nlp"
        # Set optimization_mode: "iterative", "joint_nlp", or "joint_qp" as ROS parameter:
        
        #self.declare_parameter("optimization_mode", "joint_nlp")
        #self.optimization_type = self.get_parameter("optimization_mode").get_parameter_value().string_value

        self.simple_cruise_controller = SimpleCruiseController()
        self.optimization_based_controller = OptimizationBasedController(
            self.get_logger, TIMER_PERIOD, PREDICTION_HORIZON,  self.optimization_type
        ) 

    def pedestrian_intention_state_list_callback(
        self, ped_list: PedestrianIntentionStateList
    ) -> None:
        """
        Handles incoming PedestrianIntentionStateList messages, updating vehicle and pedestrian states.

        This callback is triggered when a PedestrianIntentionStateList message is received. It processes
        the data to update state information, focusing on the relative positions and velocities of the pedestrian crossing
        and pedesrians. The information is converted to the pedestrian crossing coordinate system using
        relative kinematics.

        Args:
            ped_list (PedestrianIntentionStateList): A list of pedestrian intention states, each containing:
                - header (std_msgs/Header): The ROS message header.
                - pedestrian_states (PedestrianIntentionState[]): An array of pedestrian intention states, each with:
                    - header (std_msgs/Header): The ROS message header.
                    - id (str): The identifier of the pedestrian.
                    - intention (float64): The intention value of the pedestrian, typically ranging from 0 to 1.
                    - position (float64[2]): The x, y coordinates of the pedestrian's position.
                    - velocity (float64[2]): The x, y components of the pedestrian's velocity.

        Returns:
            None
        """

        # Check how long the PedestrianIntentionStateList is:
        num_pedestrians = len(ped_list.pedestrian_states)
        if num_pedestrians == 0:
            # No data.
            self.get_logger().error(
                "No data received, where am I? - existential crisis."
            )

        elif num_pedestrians > 0:
            # Only pedestrian crossing position is available.

            # First get the telative position and velocity of the vehicle and pedestrian to the pedestrian crossing:
            # 0 means relative to vehicle
            # 1 means relative to pedestrian crossing
            # Rule: v_P20 = v_P21 + V_10 + w_10 x rho_P21 # see Dinamika jegyzet - relativ kinematika p93

            # Since the pedestrian crossing is a fixed point, its relative position and velocity are just the
            # opposite of the vehicles relative position and velocity to the pedestrian crossing:

            # Vector from vehicle to pedestrian crossing:
            r_pedcross_0 = np.array(
                [
                    ped_list.pedestrian_states[0].position[0],
                    ped_list.pedestrian_states[0].position[1],
                ]
            )

            # Vector from pedestrian crossing to vehicle:
            r_veh_0 = -r_pedcross_0

            # Vehicle velocity vector relative to the pedestrian crossing:
            v_veh_0 = -np.array(
                [
                    ped_list.pedestrian_states[0].velocity[0],
                    ped_list.pedestrian_states[0].velocity[1],
                ]
            )

            # The orientation of the vehicle coord sys and the pedestrian crossing coord sys is the same, so
            # transformation in this case is not necessary:
            r_veh_1 = r_veh_0
            v_veh_1 = v_veh_0

            # Update the vehicle state:
            self.vehicle.state = np.array(
                [r_veh_1[0], v_veh_1[0], r_veh_1[1], v_veh_1[1]]  # x  # xd  # y
            )  # yd

            if num_pedestrians > 1:
                # Pedestrian crossing and at least 1 pedestrian data are available.
                # We assume that the first pedestrian in the list is the relevant one.
                self.is_there_a_pedestrian = True

                # Vector from vehicle to pedestrian:
                r_ped_0 = np.array(
                    [
                        ped_list.pedestrian_states[1].position[0],
                        ped_list.pedestrian_states[1].position[1],
                    ]
                )

                # Vector from pedestrian crossing to pedestrian:
                r_ped_1 = r_ped_0 - r_pedcross_0

                # Velocity vector of the pedestrian relative to the vehicle:
                v_ped_0 = np.array(
                    [
                        ped_list.pedestrian_states[1].velocity[0],
                        ped_list.pedestrian_states[1].velocity[1],
                    ]
                )

                # Velocity vector of the pedestrian relative to the pedestrian crossing:
                # (Trafo from veh coord sys to ped cros coord sys is again not necessary.)
                v_ped_1 = (
                    v_ped_0
                    + v_veh_0
                    + np.cross([0.0, 0.0, 0.0], [r_ped_0[0], r_ped_0[1], 0.0])[:2]
                )

                # Update the pedestrian state:
                self.pedestrian.update_pedestrian_state(
                    [
                        r_ped_1[0],  # x
                        v_ped_1[0],  # xd
                        r_ped_1[1],  # y
                        v_ped_1[1],  # yd
                        ped_list.pedestrian_states[1].intention,  # intention
                        ped_list.pedestrian_states[1].id,
                    ]
                )  # displaymessage

        else:
            # Something went wrong.
            self.get_logger().error("Invalid PedestrianIntentionStateList length")

    def publish_vehicle_input_message(
        self, vehicle_input: float, vehicle_display_message: str
    ) -> None:
        """
        Publishes the vehicle input message.

        This method publishes the vehicle input message with the decision result publisher.
        It sets the desired vehicle speed to 0.0, the desired vehicle acceleration to the given vehicle input,
        and the EHMI text message to the given vehicle display message. It also increments the message index
        and publishes the updated EHMI decision result message.

        Args:
            vehicle_input (float): The desired vehicle acceleration.
            vehicle_display_message (str): The EHMI text message to be displayed.
        """

        self.ehmi_decision_result_message.vehicle_speed_desired = 0.0
        self.ehmi_decision_result_message.vehicle_acceleration_desired = vehicle_input
        self.ehmi_decision_result_message.ehmi_text_message = vehicle_display_message
        self.ehmi_decision_result_message.message_index += 1
        self.publisher_decision_result.publish(self.ehmi_decision_result_message)

    def publish_system_state_prediction(self, vehicle_optimization_cat_states) -> None:
        """
        Publishes the system state prediction.

        This method retrieves the latest state prediction from the MPC problem and publishes it as a
        SystemStatesMsg. The state prediction includes the positions and velocities of the vehicle and
        pedestrian. In every step, the state prediction is saved with dstack in a matrix, so the last item
        of the last dimension contains the latest state prediction in a matrix form.
        state_prediction_matrix = vehicle_optimization_cat_states[:,:,-1]
        """

        system_states_message = SystemStatesMsg()

        if len(vehicle_optimization_cat_states.shape) == 2:
            state_prediction_matrix = vehicle_optimization_cat_states
        else:
            state_prediction_matrix = vehicle_optimization_cat_states[:, :, -1]

        x_veh = state_prediction_matrix[0, :]
        xd_veh = state_prediction_matrix[1, :]
        y_veh = state_prediction_matrix[2, :]
        yd_veh = state_prediction_matrix[3, :]
        x_ped = state_prediction_matrix[4, :]
        y_ped = state_prediction_matrix[6, :]

        for x_veh, xd_veh, y_veh, yd_veh, x_ped, y_ped in zip(
            x_veh, xd_veh, y_veh, yd_veh, x_ped, y_ped
        ):
            vehicle_state_message = SystemStateMsg()
            vehicle_state_message.x_veh = x_veh
            vehicle_state_message.xd_veh = xd_veh
            vehicle_state_message.y_veh = y_veh
            vehicle_state_message.yd_veh = yd_veh
            vehicle_state_message.x_ped = x_ped
            vehicle_state_message.y_ped = y_ped
            system_states_message.system_states.append(vehicle_state_message)

        self.publisher_vehicle_state_prediction.publish(system_states_message)

    def timer_callback(self):
        
        pedestrian_intention_decay = 0.0
        pedestrian_y_speed_smooth_decay = 0.0

        vehicle_input = 0.0
        vehicle_display_message = "---"

        if self.is_there_a_pedestrian == False:
            vehicle_input, vehicle_display_message = (
                self.simple_cruise_controller.calculate_input(
                    self.vehicle.target_state[1], self.vehicle.state[1]
                )
            )

        else:
            vehicle_in_negotiation_area = -30.0 < self.vehicle.x_position < 0.0
            pedestrian_in_negotiation_area = -6.5 < self.pedestrian.y_position < 1.5

            if vehicle_in_negotiation_area and pedestrian_in_negotiation_area:
                self.optimization_based_controller.set_states_and_targets(
                    self.vehicle,
                    self.pedestrian
                )
                vehicle_input, vehicle_display_message, pedestrian_intention_decay, pedestrian_y_speed_smooth_decay = (
                    self.optimization_based_controller.do_optimization()
                )

                self.publish_system_state_prediction(
                    self.optimization_based_controller.vehicle_optimization.cat_states
                )

            else:
                vehicle_input, vehicle_display_message = (
                    self.simple_cruise_controller.calculate_input(
                        self.vehicle.target_state[1], self.vehicle.state[1]
                    )
                )

        self.publish_vehicle_input_message(vehicle_input, vehicle_display_message)
        self.publisher_intention_decay_value.publish(
            Float64(data=pedestrian_intention_decay)
        )
        self.publisher_pedyspeed_decay_value.publish(
            Float64(data=pedestrian_y_speed_smooth_decay)
        )


def main(args=None):
    rclpy.init(args=args)

    decision_making_node_instance = DecisionMakingNode()
    rclpy.spin(decision_making_node_instance)
    decision_making_node_instance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
