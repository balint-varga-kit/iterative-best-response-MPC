import numpy as np
import time
from typing import List

from .optimization_pedestrian import PedestrianOptimization
from .optimization_vehicle import VehicleOptimization


class OptimizationBasedController:
    def __init__(self, logger, timer_period=0.2, prediction_horizon=10):
        self.logger = logger

        # Parameters:
        self.timer_period: float = timer_period
        self.prediction_horizon: int = prediction_horizon

        self.pedestrian_personal_space_radius: float = 0.5
        self.vehicle_collision_radius: float = 3.0

        self.pedestrian_wait_target_y_position: float = -2.25
        self.pedestrian_cross_target_y_position: float = 2.25
        self.pedestrian_target_y_position_intention_threshold: float = 0.5

        self.road_width: float = 3.0

        self.first_to_optimize: str = "pedestrian"  # "pedestrian" or "vehicle"

        self.break_condition_parameters = {
            "max_iterations": 5,
            "norm_threshold": 1e-5,
            "max_duration": self.timer_period * 1.5,
        }

        # Initialization:
        self.vehicle_optimization = VehicleOptimization(
            self.timer_period, self.prediction_horizon, logger
        )
        self.pedestrian_optimization = PedestrianOptimization(
            self.timer_period, self.prediction_horizon, logger
        )

        self.vehiclestate: np.ndarray = np.zeros(4)
        self.vehicletargetstate: np.ndarray = np.zeros(4)
        self.pedestrianstate: np.ndarray = np.zeros(4)
        self.pedestriantargetstate: np.ndarray = np.zeros(4)
        self.pedestrian_y_speed_smooth_decay: float = 0.0
        self.pedestrian_intention_decay: float = 0.0
        self.pedestrian_intention: float = 0.0

        self.optimization_duration_values: List[float] = []

        self.initial_system_state = np.vstack(
            (self.vehiclestate, self.pedestrianstate)
        ).reshape(-1, 1)

        self.target_system_state = np.vstack(
            (self.vehicletargetstate, self.pedestriantargetstate)
        ).reshape(-1, 1)

    def update_pedestrian_intention_decay(self):
        # Lets create an intention constant decay if not moving.
        if abs(self.pedestrianstate[2]) > 0.01:
            self.pedestrian_intention_decay = self.pedestrian_intention
        elif self.pedestrian_intention_decay > 0:
            self.pedestrian_intention_decay = max(
                0.0, self.pedestrian_intention_decay - 3 / 10 * self.timer_period
            )
            
    def update_pedestrian_speed_smooth_decay(self):

        # Update the smooth decay of the pedestrian's speed:

        slope = 1 / ((self.pedestrian_intention + 0.1) * self.timer_period*60)
        y_speed = self.pedestrianstate[2]
        y_speed_smooth_decay = self.pedestrian_y_speed_smooth_decay

        # Upwards jumping, downwards smooth decay:
        if y_speed > y_speed_smooth_decay:
            y_speed_smooth_decay = y_speed
        elif y_speed < y_speed_smooth_decay:
            y_speed_smooth_decay = max(y_speed, y_speed_smooth_decay - slope)

        self.pedestrian_y_speed_smooth_decay = y_speed_smooth_decay

    def set_states_and_targets(self, vehicle, pedestrian) -> None:

        self.vehiclestate = vehicle.state
        self.vehicletargetstate = vehicle.target_state
        self.pedestrianstate = pedestrian.state
        self.pedestriantargetstate = pedestrian.target_state
        self.pedestrian_y_speed_smooth_decay = pedestrian.y_speed_smooth_decay
        # self.pedestrian_intention_decay = pedestrian.intention_decay
        self.pedestrian_intention = pedestrian.intention

        self.initial_system_state = np.vstack(
            (self.vehiclestate, self.pedestrianstate)
        ).reshape(-1, 1)

        # But first it is important that the goal below the road is only possible if the pedestrian is not yet on the road.
        wait_target = np.array(
            [
                0.0,
                0.0,
                self.pedestrian_wait_target_y_position,
                self.pedestrian_y_speed_smooth_decay,
            ]
        )
        cross_target = np.array(
            [
                0.0,
                0.0,
                self.pedestrian_cross_target_y_position,
                self.pedestrian_y_speed_smooth_decay,
            ]
        )

        # Since why would a pedestrian step on the road if it does not want to cross.
        if self.pedestrianstate[2] < -self.road_width / 2.0:

            # Here we shall put a logic to determine the goal state of the pedestrian:
            if (
                self.pedestrian_intention_decay
                > self.pedestrian_target_y_position_intention_threshold
            ):
                self.pedestriantargetstate = cross_target
            else:
                self.pedestriantargetstate = wait_target

        else:
            self.pedestriantargetstate = cross_target

        self.target_system_state = np.vstack(
            (self.vehicletargetstate, self.pedestriantargetstate)
        ).reshape(-1, 1)

    def do_vehicle_optimization(self) -> None:
        """
        This method extracts the results of the pedestrian optimization problem and uses them to initialize the vehicle
        optimization problem. It then solves the vehicle optimization problem. The results are stored in the
        vehicle_optimization.cat_states attribute of the vehicle_optimization object.
        """

        if len(self.pedestrian_optimization.cat_states.shape) == 2:
            pedestrian_optimization_results = self.pedestrian_optimization.cat_states
        else:
            pedestrian_optimization_results = self.pedestrian_optimization.cat_states[
                :, :, -1
            ]

        self.vehicle_optimization.reinitialize_problem(
            self.initial_system_state,
            self.target_system_state,
            pedestrian_optimization_results,
            self.pedestrian_personal_space_radius,
            self.vehicle_collision_radius,
        )
        self.vehicle_optimization.solve_once()

    def do_pedestrian_optimization(self) -> None:
        """
        This method extracts the results of the vehicle optimization problem and uses them to initialize the pedestrian
        optimization problem. It then solves the pedestrian optimization problem. The results are stored in the
        pedestrian_optimization.cat_states attribute of the pedestrian_optimization object.
        """

        if len(self.vehicle_optimization.cat_states.shape) == 2:
            vehicle_optimization_results = self.vehicle_optimization.cat_states
        else:
            vehicle_optimization_results = self.vehicle_optimization.cat_states[
                :, :, -1
            ]

        self.pedestrian_optimization.reinitialize_problem(
            self.initial_system_state,
            self.target_system_state,
            vehicle_optimization_results,
            self.pedestrian_personal_space_radius,
            self.vehicle_collision_radius,
        )
        self.pedestrian_optimization.solve_once()

    def break_condition_is_met(self, iteration_counter: int, start_time: float) -> bool:
        """
        Determine whether a break condition is met.

        Args:
            iteration_counter (int): Current iteration number.
            start_time (float): Time when the optimization loop started.

        Returns:
            bool: True if the break condition is met, False otherwise.
        """

        # Condition 1: Maximum loop iterations reached.
        if iteration_counter >= self.break_condition_parameters["max_iterations"]:
            return True

        # Condition 2: The difference between the last two results is small, which means convergence is reached.
        if len(self.vehicle_optimization.cat_states.shape) == 2:
            last_result = 0
            last_last_result = 0
        else:
            last_result = self.vehicle_optimization.cat_states[:, :, -1]
            last_last_result = self.vehicle_optimization.cat_states[:, :, -2]

        difference = last_result - last_last_result
        norm = np.linalg.norm(difference, "fro")

        if norm < self.break_condition_parameters["norm_threshold"]:
            return True

        # Condition 3: The optimization takes too much time.
        if time.time() - start_time > self.break_condition_parameters["max_duration"]:
            return True

        return False

    def extract_result(self) -> tuple[float, str]:
        """
        Extracts acceleration information from the optimization result. Than maps the complete
        acceleration prediction to a display message. (Thresholds are hardcoded.)

        Returns:
            Tuple[float, str]: A tuple containing the acceleration value (in the x direction)
            and a display message indicating whether the vehicle is "Driving," "Stopping,"
            or "Stopping Abruptly."
        """

        # The full() method is converting a casadi DM object to a numpy array.
        vehicle_inputs = [self.vehicle_optimization.u.full()]
        # Then we extract the acceleration in the x direction:
        vehicle_input = vehicle_inputs[0][0][
            0
        ]  # [[x x x x ...][y y y y ...]] (strange indexing)

        # Extract the whole input prediction vector for the x axis:
        vehicle_x_inputs = vehicle_inputs[0][0]

        if np.sum(vehicle_x_inputs < -0.02) > 0.5 * len(vehicle_x_inputs):
            vehicle_display_message = "Stopping"
            if np.sum(vehicle_x_inputs < -6.0 * 0.8) > 0.8 * len(vehicle_x_inputs):
                vehicle_display_message = "Stopping Abruptly"
        else:
            vehicle_display_message = "Driving"

        return vehicle_input, vehicle_display_message

    def do_optimization(self) -> tuple[float, str]:
        """
        Perform optimization based on a specific strategy.

        This method iteratively performs optimization by alternating between optimizing
        the pedestrian and vehicle optimization problems. The optimization process continues until
        a break condition is met.

        Raises:
            ValueError: If the value of `first_to_optimize` is invalid.

        Returns:
            tuple[float, str]: A tuple containing the vehicle input and display message.
        """

        start_time = time.time()
        iteration_counter = 1
        
        self.update_pedestrian_intention_decay()
        self.update_pedestrian_speed_smooth_decay()

        while True:

            if self.first_to_optimize == "pedestrian":
                self.do_pedestrian_optimization()
                self.do_vehicle_optimization()
            elif self.first_to_optimize == "vehicle":
                self.do_vehicle_optimization()
                self.do_pedestrian_optimization()
            else:
                raise ValueError("Invalid value for first_to_optimize")

            if self.break_condition_is_met(iteration_counter, start_time):
                break
            else:
                iteration_counter += 1
            # END OF WHILE LOOP

        duration = time.time() - start_time
        self.optimization_duration_values.append(duration)
        self.optimization_duration_values = self.optimization_duration_values[-5:]
        moving_avg = sum(self.optimization_duration_values) / len(
            self.optimization_duration_values
        )  # This is for testing the duration of the optimization, currently not used.
        
        if duration > self.timer_period:
            self.logger().warning(
                f"Optimization duration exceeded self.timer_period! Timer Period: {self.timer_period}, Duration: {duration}"
            )

        vehicle_input, vehicle_display_message = self.extract_result()

        return vehicle_input, vehicle_display_message, self.pedestrian_intention_decay, self.pedestrian_y_speed_smooth_decay
