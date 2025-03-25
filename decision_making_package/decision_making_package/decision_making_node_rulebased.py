#!/usr/bin/env python

# Import Python:
import time
import numpy as np
from typing import List

import rclpy
from rclpy.node import Node
from decision_making_interface.msg import (
    # PedestrianIntentionState,
    EhmiDecisionResult,
    PedestrianIntentionStateList,
)

from crossing_gui_ros2_interfaces.msg import SimulationOutput, SystemStatesMsg, SystemStateMsg
from crossing_gui_ros2_interfaces.srv import GetGuiParam

from .vehicle_class_definition import Vehicle
from .pedestrian_class_definition import Pedestrian

# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup

# from threading import Event

# Rule Based Controller
from .utils.rule_based_controller import RuleBasedController

# Constants:
TIMER_PERIOD = 0.2
PREDICTION_HORIZON = 10

is_discounting_intention = False

np.set_printoptions(precision=4, suppress=True, linewidth=np.inf)

class DecisionMakingNode(Node):
    def __init__(self):
        super().__init__("decision_making_node")

        # ===== Creating Subscriber and Publisher ===== #
        self.subscriber_simulation_feedback = self.create_subscription(SimulationOutput, 'simulation/output', self.callback_simulation_feedback, 10)
        # self.callback_group_statelist = ReentrantCallbackGroup()
        self.ped_intention_subscription = self.create_subscription(PedestrianIntentionStateList, "relevant_pedestrian/intention_position", self.pedestrian_intention_state_list_callback, 10)#,callback_group=self.callback_group_statelist)

        self.decision_result_publisher = self.create_publisher(EhmiDecisionResult, "negotiation_node/out/decision_result", 10)
        self.publisher_vehicle_state_prediction = self.create_publisher(SystemStatesMsg, 'vehicle_state_prediction', 10)


        # self.callback_group_getguiparam = ReentrantCallbackGroup()
        self.get_gui_param_client = self.create_client(GetGuiParam, '/gui/get_gui_param')#, callback_group=self.callback_group_getguiparam)
        while not self.get_gui_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/gui/get_gui_param service not available, waiting...')

        # Initialize the decision result message:
        self.ehmi_decision_result_message= EhmiDecisionResult()
        self.ehmi_decision_result_message.vehicle_speed_desired = 0.0 # float64
        self.ehmi_decision_result_message.vehicle_acceleration_desired = 0.0 # float64
        self.ehmi_decision_result_message.ehmi_text_message = "Decision node init" # string
        self.ehmi_decision_result_message.message_index = 0 # int64

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        # Initialize the vehicle and pedestrian objects: PARAMETERS
        self.vehicle = Vehicle(5.0, 0.0, 0.0, 10.0, 0.0, 0.0, "decision node init")
        self.pedestrian = Pedestrian(40.0, 0.0, 0.0, 15.0, 0.0, 0.0, 0.0, "decision node init", TIMER_PERIOD)
        self.vehicle.target_state = np.array([0.0, 8.0, 0.0, 0.0]) # average driving speed
        self.pedestrian.target_state = np.array([0.0, 0.0, 0.0, 1.4]) # average walking speed

        self.vehicle_collision_radius = 3.0
        self.pedestrian_personal_space_radius = 0.5

        self.is_there_a_pedestrian = False
        
        self.controller = RuleBasedController()
        



    def pedestrian_intention_state_list_callback(self, ped_list: PedestrianIntentionStateList) -> None:
        """
        Callback function for handling PedestrianIntentionStateList.

        Args:
            ped_list (PedestrianIntentionStateList): List of pedestrian intention states.

        PedestrianIntentionStateList message definition:
            std_msgs/Header header
            PedestrianIntentionState[] pedestrian_states
                std_msgs/Header header
                string id
                float64 intention
                float64[2] position
                float64[2] velocity
        """

        # self.get_logger().info("PedestrianIntentionStateList received")

        # Check how long the PedestrianIntentionStateList is:
        num_pedestrians = len(ped_list.pedestrian_states)
        if num_pedestrians == 0:
            # No data.
            self.get_logger().error("No data received, where am I? - existential crisis.")

        elif num_pedestrians > 0:
            # Only pedestrian crossing position is available.

            # First get the telative position and velocity of the vehicle and pedestrian to the pedestrian crossing:
            # 0 means relative to vehicle
            # 1 means relative to pedestrian crossing
            # Rule: v_P20 = v_P21 + V_10 + w_10 x rho_P21 # see Dinamika jegyzet - relativ kinematika p93

            # Since the pedestrian crossing is a fixed point, its relative position and velocity are just the 
            # opposite of the vehicles relative position and velocity to the pedestrian crossing:

            # Vector from vehicle to pedestrian crossing:
            r_pedcross_0 = np.array([ped_list.pedestrian_states[0].position[0],
                                ped_list.pedestrian_states[0].position[1]])

            # Vector from pedestrian crossing to vehicle:
            r_veh_0 = - r_pedcross_0

            # Vehicle velocity vector relative to the pedestrian crossing:
            v_veh_0 = - np.array([ped_list.pedestrian_states[0].velocity[0],
                                ped_list.pedestrian_states[0].velocity[1]])
            
            # The orientation of the vehicle coord sys and the pedestrian crossing coord sys is the same, so
            # transformation in this case is not necessary:
            r_veh_1 = r_veh_0
            v_veh_1 = v_veh_0

            # Update the vehicle state:
            self.vehicle.state = np.array([ r_veh_1[0],     # x
                                            v_veh_1[0],     # xd
                                            r_veh_1[1],     # y
                                            v_veh_1[1] ])   # yd

            if num_pedestrians > 1:
                # Pedestrian crossing and at least 1 pedestrian data are available.
                # We assume that the first pedestrian in the list is the relevant one.
                self.is_there_a_pedestrian = True

                 # Vector from vehicle to pedestrian:
                r_ped_0 = np.array([ped_list.pedestrian_states[1].position[0],
                                    ped_list.pedestrian_states[1].position[1]])

                # Vector from pedestrian crossing to pedestrian:
                r_ped_1 = r_ped_0 - r_pedcross_0
                
                # Velocity vector of the pedestrian relative to the vehicle:
                v_ped_0 = np.array([ped_list.pedestrian_states[1].velocity[0],
                                    ped_list.pedestrian_states[1].velocity[1]])

                # Velocity vector of the pedestrian relative to the pedestrian crossing:
                # (Trafo from veh coord sys to ped cros coord sys is again not necessary.)
                v_ped_1 = v_ped_0 + v_veh_0 + np.cross([0.0, 0.0, 0.0], [r_ped_0[0], r_ped_0[1], 0.0])[:2]

                # Update the pedestrian state:
                self.pedestrian.update_pedestrian_state([r_ped_1[0], # x
                                                        v_ped_1[0], # xd
                                                        r_ped_1[1], # y
                                                        v_ped_1[1], # yd
                                                        ped_list.pedestrian_states[1].intention, # intention
                                                        ped_list.pedestrian_states[1].id]) # displaymessage

        else:
            # Something went wrong.
            self.get_logger().error("Invalid PedestrianIntentionStateList length")

    def callback_simulation_feedback(self, msg: SimulationOutput) -> None:
        """
        Saves the output of the simulation node locally for further usage.
        Currently not used and should not be used in the future.
        """
        pass

    def publish_vehicle_input(self):
        """
        Publishes the calculated input for the vehicle.

        This method retrieves the latest input for the vehicle from the vehicle optimization problem and
        publishes it as a EhmiDecisionResult message. The input includes the desired speed and acceleration
        of the vehicle.

        EhmiDecisionResult message definition:
            float64 vehicle_speed_desired 
            float64 vehicle_acceleration_desired
            string ehmi_text_message
            int64 message_index
        """
        

        self.ehmi_decision_result_message.vehicle_speed_desired = 0.0
        self.ehmi_decision_result_message.vehicle_acceleration_desired = self.vehicle.x_acceleration
        self.ehmi_decision_result_message.ehmi_text_message = self.vehicle.displaymessage
        self.ehmi_decision_result_message.message_index += 1

        self.decision_result_publisher.publish(self.ehmi_decision_result_message)
        
        self.get_logger().info("Vehicle acceleration: " + str(self.vehicle.x_acceleration))



    def intention_discounting(self):
        is_discounting = abs(self.pedestrian.y_position - self.pedestrian.y_position) > 1.5 and abs(self.pedestrian.y_speed) < 0.5
        if is_discounting:
            self.count_para += 1
        else:
            self.count_para = 0
        self.pedestrian.intention *= pow(0.95, self.count_para * 0.5)


    def timer_callback(self):


        if is_discounting_intention:
            self.intention_discounting()

        t1 = time.time()
        self.vehicle.x_acceleration, self.vehicle.displaymessage = self.controller.eval(self.pedestrian, self.vehicle, self.get_logger)
        t2 = time.time()
        # self.get_logger().info(f'cost time: {t2 - t1}')

        self.publish_vehicle_input()





def main(args=None):
    rclpy.init(args=args)

    decision_making_node_instance = DecisionMakingNode()
    rclpy.spin(decision_making_node_instance)
    decision_making_node_instance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


