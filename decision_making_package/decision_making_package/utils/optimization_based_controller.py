import numpy as np
import time
from typing import List
import casadi as ca

# Import JointOptimization classes.
from .joint_optimization import JointOptimizationNLP, JointOptimizationQP
# Import original iterative modules.
from .optimization_pedestrian import PedestrianOptimization
from .optimization_vehicle import VehicleOptimization

class OptimizationBasedController:
    def __init__(self, logger, timer_period=0.2, prediction_horizon=10, optimization_mode="joint_nlp"):
        self.logger = logger
        self.timer_period: float = timer_period
        self.prediction_horizon: int = prediction_horizon

        self.pedestrian_personal_space_radius: float = 0.5
        self.vehicle_collision_radius: float = 3.0

        self.pedestrian_wait_target_y_position: float = -2.25
        self.pedestrian_cross_target_y_position: float = 2.25
        self.pedestrian_target_y_position_intention_threshold: float = 0.5

        self.road_width: float = 3.0

        # Mode selection: "iterative", "joint_nlp", or "joint_qp"
        self.optimization_mode = optimization_mode

        if self.optimization_mode == "iterative":
            self.vehicle_optimization = VehicleOptimization(
                self.timer_period, self.prediction_horizon, logger
            )
            self.pedestrian_optimization = PedestrianOptimization(
                self.timer_period, self.prediction_horizon, logger
            )
            # Set default for first_to_optimize.
            self.first_to_optimize = "pedestrian"
        elif self.optimization_mode == "joint_nlp":
            self.vehicle_model = lambda x, u: x + self.timer_period * ca.vertcat(x[1], u, x[3], 0)
            self.pedestrian_model = lambda x, u: x + self.timer_period * ca.vertcat(x[1], u[0], x[3], u[1])
            self.joint_optimization = JointOptimizationNLP(
                N=self.prediction_horizon,
                dt=self.timer_period,
                vehicle_model=self.vehicle_model,
                pedestrian_model=self.pedestrian_model,
                vehicle_state_dim=4,
                vehicle_ctrl_dim=1,
                ped_state_dim=4,
                ped_ctrl_dim=2,
                D_safe=self.vehicle_collision_radius+self.pedestrian_personal_space_radius
            )
            # For backward compatibility, assign joint optimizer to vehicle_optimization.
            self.vehicle_optimization = self.joint_optimization
        elif self.optimization_mode == "joint_qp":
            self.vehicle_model = lambda x, u: x + self.timer_period * ca.vertcat(x[1], u, x[3], 0)
            self.pedestrian_model = lambda x, u: x + self.timer_period * ca.vertcat(x[1], u[0], x[3], u[1])
            self.joint_optimization = JointOptimizationQP(
                N=self.prediction_horizon,
                dt=self.timer_period,
                vehicle_model=self.vehicle_model,
                pedestrian_model=self.pedestrian_model,
                vehicle_state_dim=4,
                vehicle_ctrl_dim=1,
                ped_state_dim=4,
                ped_ctrl_dim=2,
                D_safe=self.vehicle_collision_radius
            )
            self.vehicle_optimization = self.joint_optimization
        else:
            raise ValueError("Invalid optimization_mode provided. Use 'iterative', 'joint_nlp', or 'joint_qp'.")

        self.vehiclestate: np.ndarray = np.zeros(4)
        self.vehicletargetstate: np.ndarray = np.zeros(4)
        self.pedestrianstate: np.ndarray = np.zeros(4)
        self.pedestriantargetstate: np.ndarray = np.zeros(4)
        self.pedestrian_y_speed_smooth_decay: float = 0.0
        self.pedestrian_intention_decay: float = 0.0
        self.pedestrian_intention: float = 0.0

        self.optimization_duration_values: List[float] = []

        self.initial_system_state = np.vstack((self.vehiclestate, self.pedestrianstate)).reshape(-1, 1)
        self.target_system_state = np.vstack((self.vehicletargetstate, self.pedestriantargetstate)).reshape(-1, 1)

    def update_pedestrian_intention_decay(self):
        if abs(self.pedestrianstate[2]) > 0.01:
            self.pedestrian_intention_decay = self.pedestrian_intention
        elif self.pedestrian_intention_decay > 0:
            self.pedestrian_intention_decay = max(0.0, self.pedestrian_intention_decay - 3 / 10 * self.timer_period)
            
    def update_pedestrian_speed_smooth_decay(self):
        slope = 1 / ((self.pedestrian_intention + 0.1) * self.timer_period * 60)
        y_speed = self.pedestrianstate[2]
        y_speed_smooth_decay = self.pedestrian_y_speed_smooth_decay
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
        self.pedestrian_intention = pedestrian.intention
        self.initial_system_state = np.vstack((self.vehiclestate, self.pedestrianstate)).reshape(-1, 1)
        wait_target = np.array([0.0, 0.0, self.pedestrian_wait_target_y_position, self.pedestrian_y_speed_smooth_decay])
        cross_target = np.array([0.0, 0.0, self.pedestrian_cross_target_y_position, self.pedestrian_y_speed_smooth_decay])
        if self.pedestrianstate[2] < -self.road_width / 2.0:
            if self.pedestrian_intention_decay > self.pedestrian_target_y_position_intention_threshold:
                self.pedestriantargetstate = cross_target
            else:
                self.pedestriantargetstate = wait_target
        else:
            self.pedestriantargetstate = cross_target
        self.target_system_state = np.vstack((self.vehicletargetstate, self.pedestriantargetstate)).reshape(-1, 1)

    def extract_result_from_joint(self, Uv_opt) -> tuple[float, str]:
        if np.ndim(Uv_opt) == 1:
            vehicle_input = Uv_opt[0]
            vehicle_x_inputs = Uv_opt
        else:
            vehicle_input = Uv_opt[0, 0]
            vehicle_x_inputs = Uv_opt[0, :]
        if np.sum(vehicle_x_inputs < -0.02) > 0.5 * len(vehicle_x_inputs):
            vehicle_display_message = "Stopping"
            if np.sum(vehicle_x_inputs < -6.0 * 0.8) > 0.8 * len(vehicle_x_inputs):
                vehicle_display_message = "Stopping Abruptly"
        else:
            vehicle_display_message = "Driving"
        return vehicle_input, vehicle_display_message

    def extract_result(self) -> tuple[float, str]:
        vehicle_inputs = [self.vehicle_optimization.u.full()]
        vehicle_input = vehicle_inputs[0][0][0]
        vehicle_x_inputs = vehicle_inputs[0][0]
        if np.sum(vehicle_x_inputs < -0.02) > 0.5 * len(vehicle_x_inputs):
            vehicle_display_message = "Stopping"
            if np.sum(vehicle_x_inputs < -6.0 * 0.8) > 0.8 * len(vehicle_x_inputs):
                vehicle_display_message = "Stopping Abruptly"
        else:
            vehicle_display_message = "Driving"
        return vehicle_input, vehicle_display_message

    def break_condition_is_met(self, iteration_counter: int, start_time: float) -> bool:
        if iteration_counter >= 5:
            return True
        if len(self.vehicle_optimization.cat_states.shape) == 2:
            last_result = 0
            last_last_result = 0
        else:
            last_result = self.vehicle_optimization.cat_states[:, :, -1]
            last_last_result = self.vehicle_optimization.cat_states[:, :, -2]
        difference = last_result - last_last_result
        norm = np.linalg.norm(difference, "fro")
        if norm < 1e-5:
            return True
        if time.time() - start_time > self.timer_period * 1.5:
            return True
        return False

    def do_vehicle_optimization(self) -> None:
        if len(self.pedestrian_optimization.cat_states.shape) == 2:
            pedestrian_optimization_results = self.pedestrian_optimization.cat_states
        else:
            pedestrian_optimization_results = self.pedestrian_optimization.cat_states[:, :, -1]
        self.vehicle_optimization.reinitialize_problem(
            self.initial_system_state,
            self.target_system_state,
            pedestrian_optimization_results,
            self.pedestrian_personal_space_radius,
            self.vehicle_collision_radius,
        )
        self.vehicle_optimization.solve_once()

    def do_pedestrian_optimization(self) -> None:
        if len(self.vehicle_optimization.cat_states.shape) == 2:
            vehicle_optimization_results = self.vehicle_optimization.cat_states
        else:
            vehicle_optimization_results = self.vehicle_optimization.cat_states[:, :, -1]
        self.pedestrian_optimization.reinitialize_problem(
            self.initial_system_state,
            self.target_system_state,
            vehicle_optimization_results,
            self.pedestrian_personal_space_radius,
            self.vehicle_collision_radius,
        )
        self.pedestrian_optimization.solve_once()

    def do_optimization(self) -> tuple[float, str, float, float]:
        start_time = time.time()
        if self.optimization_mode in ["joint_nlp", "joint_qp"]:
            self.update_pedestrian_intention_decay()
            self.update_pedestrian_speed_smooth_decay()
            Uv_opt, Up_opt = self.joint_optimization.do_optimization(self.initial_system_state, self.target_system_state)
            vehicle_input, vehicle_display_message = self.extract_result_from_joint(Uv_opt)
        else:
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
            vehicle_input, vehicle_display_message = self.extract_result()
        
        duration = time.time() - start_time
        self.optimization_duration_values.append(duration)
        self.optimization_duration_values = self.optimization_duration_values[-5:]
        moving_avg = sum(self.optimization_duration_values) / len(self.optimization_duration_values)
        
        if duration > self.timer_period:
            self.logger().warning(f"Optimization duration exceeded self.timer_period! Timer Period: {self.timer_period}, Duration: {duration}")
        
        return (vehicle_input, vehicle_display_message,
                self.pedestrian_intention_decay, self.pedestrian_y_speed_smooth_decay)
