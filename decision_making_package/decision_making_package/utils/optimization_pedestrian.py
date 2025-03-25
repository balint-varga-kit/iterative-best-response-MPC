from time import time
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import matplotlib.pyplot as plt

class PedestrianOptimization:
    def __init__(self, step_horizon_passed, N_passed, logger):
        # Initialize your optimization problem here

        self.get_logger = logger

        self.step_horizon = step_horizon_passed  # time between steps in seconds (0.01 sec)
        self.N = N_passed             # number of look ahead steps (10)

        # specs
        x_veh_init = -50.0
        y_veh_init = 0
        xd_veh_init = 0
        yd_veh_init = 0
        x_ped_init = 0.0
        xd_ped_init = 0
        y_ped_init = -15.0
        yd_ped_init = 0
        state_init = ca.DM([x_veh_init,
                            xd_veh_init,
                            y_veh_init,
                            yd_veh_init,
                            x_ped_init,
                            xd_ped_init,
                            y_ped_init,
                            yd_ped_init])

        x_veh_target = 0
        y_veh_target = 0
        xd_veh_target = 8.0
        yd_veh_target = 0
        x_ped_target = 0
        xd_ped_target = 0
        y_ped_target = 10
        yd_ped_target = 0
        state_target = ca.DM([x_veh_target,
                              xd_veh_target,
                              y_veh_target,
                              yd_veh_target,
                              x_ped_target,
                              xd_ped_target,
                              y_ped_target,
                              yd_ped_target])

        ax_max = 0.0
        ax_min = 0.0
        ay_max = +0.5*15
        ay_min = -0.5*15

        self.t0 = 0
        pedestrian_personal_space_radius = 15.0 # init ped state
        vehicle_collision_radius = 15.0

        # =================== Define symbolic variables: ======================================

        # State:
        x_veh = ca.SX.sym('x_veh')
        y_veh = ca.SX.sym('y_veh')
        xd_veh = ca.SX.sym('xd_veh')
        yd_veh = ca.SX.sym('yd_veh')
        x_ped = ca.SX.sym('x_ped')
        xd_ped = ca.SX.sym('xd_ped')
        y_ped = ca.SX.sym('y_ped')
        yd_ped = ca.SX.sym('yd_ped')
        states = ca.vertcat(
            x_veh,
            xd_veh,
            y_veh,
            yd_veh,
            x_ped,
            xd_ped,
            y_ped,
            yd_ped
        )
        self.n_states = states.numel()

        vehicle_optimization_results = ca.DM.zeros(self.n_states, self.N + 1 ) # args

        # Control:
        ax_ped = ca.SX.sym('ax_ped')
        ay_ped = ca.SX.sym('ay_ped')
        controls = ca.vertcat(
            ax_ped,
            ay_ped
        )
        self.n_controls = controls.numel()

        # Matrix containing all states over all time steps +1 (each column is a state vector):
        # [ x0[0], x0[1], x0[2], ...]
        # [ x1[0]  x1[1]  x1[2]     ]
        X = ca.SX.sym('X', self.n_states, self.N + 1 )

        # Matrix containing all control actions over all time steps (each column is an action vector):
        # [ u0[0], u0[1], u0[2], ...]
        # [ u1[0]  u1[1]  u1[2]     ]
        U = ca.SX.sym('U', self.n_controls, self.N)

        # Coloumn vector for storing initial state and target state and other things:
        # [x0_init, x1_init ... , x0_target, x1_target ...]^T
        # P = ca.SX.sym('P', self.n_states + self.n_states + 1 + 1)

        p0 = ca.SX.sym('p0', self.n_states)
        p1 = ca.SX.sym('p1', self.n_states)
        p2 = ca.SX.sym('p2')
        p3 = ca.SX.sym('p3')
        p4 = ca.SX.sym('p4', self.n_states, self.N + 1)
        p5 = ca.SX.sym('p5')

        P = {'p_init_sys_state': p0, 'p_target_sys_state': p1, 'p2': p2, 'p3': p3, 'p_vehicle_optimization_results': p4}

        # State weight matrix:
        Q_x_veh = 0
        Q_xd_veh = 0
        Q_y_veh = 0
        Q_yd_veh = 0
        Q_x_ped = 10
        Q_xd_ped = 10
        Q_y_ped = 10
        Q_yd_ped = 10
        Q = ca.diagcat(Q_x_veh, Q_xd_veh, Q_y_veh, Q_yd_veh, Q_x_ped, Q_xd_ped, Q_y_ped, Q_yd_ped)
        Q = ca.diagcat(Q_x_ped, Q_xd_ped, Q_y_ped, Q_yd_ped)

        # Control weight matrix:
        R1 = 0
        R2 = 0
        R = ca.diagcat(R1, R2)

        # =================== Dynamic equations: (right hand side) ======================================
        # xd = [0 1]*x + [0]*a
        #      [0 0]     [1]
        RHS = ca.vertcat(0.0, # not needed comes from vehicle optimization
                        0.0, # not needed comes from vehicle optimization
                        0.0, # not needed comes from vehicle optimization
                        0.0, # not needed comes from vehicle optimization
                        xd_ped,
                        ax_ped,
                        yd_ped,
                        ay_ped)
        # Function to map controls to state derivative: xd = f(x,u)
        self.f = ca.Function('f', [states, controls], [RHS])


        # =================== Build the LQ cost function: ======================================
        # (x-x_veh_target)'*Q*(x-x_veh_target) + (u)'*R*(u
        cost_fn = 0
        for k in range(self.N):
            current_state = X[:, k]
            current_control = U[:, k]

            cost_fn = cost_fn + (
                + (current_state[4:8] - P['p_target_sys_state'][4:8]).T @ Q @ (current_state[4:8] - P['p_target_sys_state'][4:8])
                + current_control.T @ R @ current_control

            )


        # =================== eq constraint: Initial state ======================================
        # (first state is known from measurement)
        # At the same time this initializes g, lbg and ubg, from now on use ca.vertcat() to add constraints.
        g = X[:, 0] - P['p_init_sys_state'] 
        # this creates a 4x1 column vector from g

        lbg = ca.DM.zeros((self.n_states,1))
        ubg = ca.DM.zeros((self.n_states,1))

        # =================== eq constraint: Dynamic equations ======================================
        # (multiple shooting)
        # g[1] ... g[N]
        numerical_integration_method = "runge_kutta_4" # euler, runge_kutta_4
        for k in range(self.N):
            current_state = X[:, k]
            current_control = U[:, k]
            next_state = X[:, k+1]

            if numerical_integration_method == "runge_kutta_4":
                k1 = self.f(current_state, current_control)
                k2 = self.f(current_state + self.step_horizon/2*k1, current_control)
                k3 = self.f(current_state + self.step_horizon/2*k2, current_control)
                k4 = self.f(current_state + self.step_horizon * k3, current_control)
                predicted_next_state = current_state + (self.step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

            elif numerical_integration_method == "euler":
                predicted_next_state = current_state + self.step_horizon * self.f(current_state, current_control)

            # We just need to change the vehicle part of the next predicted state to the 
            # parameters we got from the vehicle optimization problem.
            predicted_next_state[:4] = P['p_vehicle_optimization_results'][:4, k+1]

            # self.get_logger().info(f"predicted_next_state: {predicted_next_state}")

            g = ca.vertcat(g, next_state - predicted_next_state)
            lbg = ca.vertcat(lbg, ca.DM.zeros((self.n_states,1)))
            ubg = ca.vertcat(ubg, ca.DM.zeros((self.n_states,1)))

        # =================== Extra constraints: ======================================
        # The pedestrian does not want to die!
        # Extract current state, extract current pedestrian position, and compare it to the initial
        # vehicle position.
        for k in range(self.N+1):
            current_state = X[:, k]
            x_ped = current_state[4]
            y_ped = current_state[6]
            g = ca.vertcat(g, ca.power( x_ped - P['p_init_sys_state'][0] ,2) + ca.power( y_ped - P['p_init_sys_state'][2] ,2)- ca.power(P['p2'] + 1.5, 2))
            lbg = ca.vertcat(lbg, 0 )
            ubg = ca.vertcat(ubg, ca.inf)
            
            # One could add the constraint I am using in the vehicle here as well, such that the vehicle is 3 circles not one big circle.

        # The pedestrian wants to avoid the vehicle in the future as well:
        # But lets say the pedestrian cannot predict the movement of the vehicle, so this is not used now.
        # Compares the pedestrian position to the vehicle position in every timestep.
        # for k in range(self.N+1):
        #     current_state = X[:, k]
        #     x_veh = current_state[0]
        #     y_veh = current_state[2]
        #     x_ped = current_state[4]
        #     y_ped = current_state[6]
        #     g = ca.vertcat(g, ca.power( x_veh - x_ped ,2) + ca.power( y_veh - y_ped ,2) - ca.power(P['p2'] + P['p3'], 2))
        #     lbg = ca.vertcat(lbg, 0 )
        #     ubg = ca.vertcat(ubg, ca.inf)

        # =================== Bound constraints of optimization variables ======================================
        # We need the optimization variables as one column vector:
        # (first, reshape the symbolic matrices to column vectors, then concatenate them vertically)
        # It will look like this:
        # [x0[0], x1[0], x0[1], x1[1], ..., x0[k], x1[k], ... , x0[N], x1[N], u0[0], u1[0], ...]'
        # or: [x[0]', x[1]', ... x[N]', u[0]', u[1]', ... u[N-1]']'
        OPT_variables = ca.vertcat(
            X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            U.reshape((-1, 1))
        )

        # Initialize the bounds of the optimization variables:
        # (N+1) * state_dimension + N * control_dimension
        lbx = ca.DM.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))
        ubx = ca.DM.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))

        # Lower bound of state in all timesteps:
        # syntax: [from (inclusive) : to (exclusive) : step]
        lbx[0: self.n_states*(self.N+1): self.n_states] = -100.0   # x_veh lower bound
        lbx[1: self.n_states*(self.N+1): self.n_states] = 0.0       # xd_veh lower bound
        lbx[2: self.n_states*(self.N+1): self.n_states] = -100.0   # y_veh lower bound
        lbx[3: self.n_states*(self.N+1): self.n_states] = 0.0       # yd_veh lower bound
        lbx[4: self.n_states*(self.N+1): self.n_states] = -100.0   # x_ped lower bound
        lbx[5: self.n_states*(self.N+1): self.n_states] = -2.0   # xd_ped lower bound
        lbx[6: self.n_states*(self.N+1): self.n_states] = -100.0   # y_ped lower bound
        lbx[7: self.n_states*(self.N+1): self.n_states] = -2.0   # yd_ped lower bound


        # Upper bound of state in all timesteps:
        ubx[0: self.n_states*(self.N+1): self.n_states] = 100.0      # x upper bound # map should be 200x200 m around ped cros
        ubx[1: self.n_states*(self.N+1): self.n_states] = 10.0      # xd upper bound
        ubx[2: self.n_states*(self.N+1): self.n_states] = 100.0      # y upper bound
        ubx[3: self.n_states*(self.N+1): self.n_states] = 10.0      # yd upper bound
        ubx[4: self.n_states*(self.N+1): self.n_states] = 100.0      # x_ped upper bound
        ubx[5: self.n_states*(self.N+1): self.n_states] = 2.0     # xd_ped upper bound ca. 7 km/h 
        ubx[6: self.n_states*(self.N+1): self.n_states] = 100.0      # y_ped upper bound
        ubx[7: self.n_states*(self.N+1): self.n_states] = 2.0      # yd_ped upper bound

        # Lower bound of control in all timesteps:
        lbx[self.n_states*(self.N+1)::2] = ax_min                  # a lower bound for all a
        lbx[self.n_states*(self.N+1)+1::2] = ay_min                  # a lower bound for all a

        # Upper bound of control in all timesteps:
        ubx[self.n_states*(self.N+1)::2] = ax_max                  # a upper bound for all a
        ubx[self.n_states*(self.N+1)+1::2] = ay_max                  # a upper bound for all a


        # =================== Nonlinear programming solver ======================================

        concatenated_vector, indices = self.concat_sx_dict(P)

        # Problem formulation:
        nlp_prob = {
            'f': cost_fn,
            'x': OPT_variables,
            'g': g,
            'p': concatenated_vector
        }

        # Solver options:
        opts = {
            # https://web.casadi.org/python-api/#nlp
            # Here are the options listed
            #'expand': False,  # (False) if problem is big, True could make it faster
            #'verbose': False,  # (False) if True, prints out a lot of information
            'print_time': True,
            # ... and many more options, see the link above
            'ipopt': {
                # Here are options from the second big table on the site under ipopt.
                # Explanation for the options can be found in the IPOPT documentation.
                # Not all options are available with casadi python wrapper, only which is listed in link above.
                #'tol': 1e-8, # (1e-8)
                'max_iter': 2000, # (3000)
                'print_level': 0, # (5)
                'acceptable_tol': 1e-8, # (1e-6) it was 1e-8
                'acceptable_obj_change_tol': 1e-6, # (1e+20) it was 1e-6
                # with default options, multipliers for the decision variables are wrong for equality constraints.
                # Change the 'fixed_variable_treatment' to 'make_constraint' or 'relax_bounds' to obtain correct results.
                'fixed_variable_treatment': 'make_constraint',  # (make_parameter)|make_parameter_nodual|make_constraint|relax_bounds
            }
        }

        # Initialize solver:
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        # Initialize the states with the initial state:
        self.X0 = ca.repmat(state_init, 1, self.N+1) 

        # Initialize the controls with zeros:
        self.u0 = ca.DM.zeros((self.n_controls, self.N)) 
        self.u = self.u0
        # self.u = Contains whole input solution
        # self.u0 = initial guess for the next run

        # =================== Initialize the argument list ======================================
        # This is the only thing which should change in every loop iteration:

        # Bounds on optimization variables:
        self.args = {
            'x0' : ca.vertcat(
                    ca.reshape(self.X0, self.n_states*(self.N+1), 1),
                    ca.reshape(self.u0, self.n_controls*self.N, 1)
                    ),
            'lbg': lbg,  # constraints lower bound
            'ubg': ubg,  # constraints upper bound
            'lbx': lbx,
            'ubx': ubx,
            'p'  : ca.vertcat(
                    state_init,    # current state
                    state_target,   # target state
                    ca.reshape(vehicle_optimization_results, self.n_states*(self.N+1), 1), # vehicle optimization results
                    pedestrian_personal_space_radius,
                    vehicle_collision_radius,
                    0.5
                    )
        }

        # =================== Initialize other things ======================================
        self.t = ca.DM(self.t0)
        self.mpc_iter = 0
        self.cat_states = self.DM2Arr(self.X0) # to store all state solutions (all timesteps)
        self.cat_controls = self.DM2Arr(self.u0[:, 0]) # to store all control solutions (only first timestep)
        self.time_to_solve = np.array([[0]])

    def reinitialize_problem(self, state_init, state_target, vehicle_optimization_results, pedestrian_personal_space_radius, vehicle_collision_radius):

        # We need to create a column vector from the vehicle_optimization_results:

        # Update the problem parameters:
        # !!! ORDER !!!
        self.args['p'] = ca.vertcat(
            state_init,
            state_target,
            pedestrian_personal_space_radius,
            vehicle_collision_radius,
            vehicle_optimization_results.reshape(-1, 1, order='F'),
        )

        # Update the initial guess for the state in original normal matrix form. (not column vector)
        # we initialize with the last solution, shifted by one step, and duplicate the last state.
        self.X0 = ca.horzcat(
            self.X0[:, 1:], # cut off the first column
            ca.reshape(self.X0[:, -1], -1, 1) # duplicate the last column
        )

        # Update the initial guess for the state in original normal matrix form. (not column vector)
        # we initialize with the last solution, shifted by one step, and duplicate the last state.
        self.u0 = ca.horzcat(
            self.u[:, 1:], # cut off the first column
            ca.reshape(self.u[:, -1], -1, 1) # duplicate the last column
        )

        self.args['x0'] = ca.vertcat(
            ca.reshape(self.X0, self.n_states*(self.N+1), 1),
            ca.reshape(self.u0, self.n_controls*self.N, 1)
        )

        self.t0 = self.t0 + self.step_horizon # ???

    def DM2Arr(self, dm):
        return np.array(dm.full())

    def solve_once(self):
        # Solve the optimization problem for one iteration

        # Start timer:
        t1 = time()

        # Calculate solution of the problem:
        sol = self.solver(
            x0=self.args['x0'],
            lbx=self.args['lbx'],
            ubx=self.args['ubx'],
            lbg=self.args['lbg'],
            ubg=self.args['ubg'],
            p=self.args['p']
        )
        # Extract the solution in proper matrix form: (x xd y yd)
        self.X0 = ca.reshape(sol['x'][: self.n_states * (self.N+1)], self.n_states, self.N+1)
        self.u = ca.reshape(sol['x'][self.n_states * (self.N + 1):], self.n_controls, self.N)

        # self.get_logger().info(f"Second row of the resulting states: {self.X0[1, :]}")

        # Store the whole solution matrix for the states:
        self.cat_states = np.dstack((
            self.cat_states,
            self.DM2Arr(self.X0)
        ))

        # Store the first control action:
        self.cat_controls = np.vstack((
            self.cat_controls,
            self.DM2Arr(self.u[:, 0])
        ))

        # Store initialization time:
        self.t = np.vstack((
            self.t,
            self.t0
        ))

        # End timer:
        t2 = time()

        # self.get_logger().info("Time to solve: " + str(t2-t1))

        # Store time to solve the problem:
        self.time_to_solve = np.vstack((
            self.time_to_solve,
            t2-t1
        ))

        # Increment the iteration:
        self.mpc_iter = self.mpc_iter + 1

    def concat_sx_dict(self, sx_dict):
        """
        Concatenates the values of a dictionary into a single vector and returns the concatenated vector along with the indices of each value.

        Args:
            sx_dict (dict): A dictionary containing values to be concatenated.

        Returns:
            tuple: A tuple containing the concatenated vector and a dictionary where the keys are the keys of the
            original dictionary and the values are tuples of start and end indices (start,end).
        """
        concatenated_vector = ca.vertcat(*[ca.reshape(val, -1, 1) if val.numel() > 1 else val for val in sx_dict.values()])
        indices = {}
        start_index = 0

        for key, val in sx_dict.items():
            indices[key] = (start_index, start_index + val.numel())
            start_index += val.numel()

        return concatenated_vector, indices


# Example usage:
if __name__ == '__main__':
    mpc_problem = PedestrianOptimization()
    # mpc_problem.reinitialize(x_veh_init, xd_veh_init, x_veh_target, xd_veh_target)
    mpc_problem.solve_once()
