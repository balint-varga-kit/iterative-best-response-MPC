# joint_optimization.py
import casadi as ca
import numpy as np

class JointOptimizationNLP:
    def __init__(self, N, dt, vehicle_model, pedestrian_model, 
                 vehicle_state_dim, vehicle_ctrl_dim,
                 ped_state_dim, ped_ctrl_dim,
                 D_safe):
        """
        Joint NLP optimization: all dynamics and cost are modeled nonlinearly.
        The collision avoidance constraint is enforced as originally defined.
        """
        self.N = N
        self.dt = dt
        self.D_safe = D_safe
        
        # Create an Opti instance.
        self.opti = ca.Opti()
        
        # Decision variables for vehicle and pedestrian trajectories.
        self.Xv = self.opti.variable(vehicle_state_dim, N+1)
        self.Uv = self.opti.variable(vehicle_ctrl_dim, N)
        self.Xp = self.opti.variable(ped_state_dim, N+1)
        self.Up = self.opti.variable(ped_ctrl_dim, N)
        
        # Parameters for initial conditions.
        self.x0_v = self.opti.parameter(vehicle_state_dim, 1)
        self.x0_p = self.opti.parameter(ped_state_dim, 1)
        self.opti.subject_to(self.Xv[:, 0] == self.x0_v)
        self.opti.subject_to(self.Xp[:, 0] == self.x0_p)
        
        # Build cost function and constraints.
        total_cost = 0
        Q_vehicle = ca.DM.eye(vehicle_state_dim)
        R_vehicle = ca.DM.eye(vehicle_ctrl_dim)
        Q_ped = ca.DM.eye(ped_state_dim)
        R_ped = ca.DM.eye(ped_ctrl_dim)
        
        for k in range(N):
            x_v = self.Xv[:, k]
            u_v = self.Uv[:, k]
            x_p = self.Xp[:, k]
            u_p = self.Up[:, k]
            
            total_cost += ca.mtimes([x_v.T, Q_vehicle, x_v]) + ca.mtimes([u_v.T, R_vehicle, u_v])
            total_cost += ca.mtimes([x_p.T, Q_ped, x_p]) + ca.mtimes([u_p.T, R_ped, u_p])
            
            # Dynamics constraints (using provided nonlinear models).
            x_v_next = vehicle_model(x_v, u_v)
            self.opti.subject_to(self.Xv[:, k+1] == x_v_next)
            x_p_next = pedestrian_model(x_p, u_p)
            self.opti.subject_to(self.Xp[:, k+1] == x_p_next)
            
            # Collision avoidance constraint:
            # (x_v[0]-x_p[0])^2 + (x_v[1]-x_p[1])^2 >= D_safe^2
            dx = self.Xv[0, k] - self.Xp[0, k]
            dy = self.Xv[1, k] - self.Xp[1, k]
            self.opti.subject_to(dx*dx + dy*dy >= D_safe**2)
            
        # Terminal cost.
        x_v_final = self.Xv[:, N]
        x_p_final = self.Xp[:, N]
        total_cost += ca.mtimes([x_v_final.T, Q_vehicle, x_v_final])
        total_cost += ca.mtimes([x_p_final.T, Q_ped, x_p_final])
        
        self.opti.minimize(total_cost)
        
        # Solver options.
        opts = {"print_time": False, "ipopt": {"print_level": 0, "max_iter": 500, "tol": 1e-4}}
        self.opti.solver('ipopt', opts)
        self.prev_solution = None

    def do_optimization(self, init_state, target_state=None):
        vehicle_state_dim = self.x0_v.size()[0]
        init_vehicle = init_state[:vehicle_state_dim]
        init_ped = init_state[vehicle_state_dim:]
        self.opti.set_value(self.x0_v, init_vehicle)
        self.opti.set_value(self.x0_p, init_ped)
        
        if self.prev_solution is not None:
            self.opti.set_initial(self.Xv, self.prev_solution['Xv'])
            self.opti.set_initial(self.Uv, self.prev_solution['Uv'])
            self.opti.set_initial(self.Xp, self.prev_solution['Xp'])
            self.opti.set_initial(self.Up, self.prev_solution['Up'])
        
        sol = self.opti.solve()
        Uv_opt = sol.value(self.Uv)
        Up_opt = sol.value(self.Up)
        self.prev_solution = {
            'Xv': sol.value(self.Xv),
            'Uv': Uv_opt,
            'Xp': sol.value(self.Xp),
            'Up': Up_opt
        }
        return Uv_opt, Up_opt

    @property
    def cat_states(self):
        """
        Return the combined (vehicle; pedestrian) state trajectory.
        Expected shape is (vehicle_state_dim+ped_state_dim, N+1).
        """
        if self.prev_solution is not None:
            return np.vstack((self.prev_solution['Xv'], self.prev_solution['Xp']))
        else:
            return np.empty((0,))

class JointOptimizationQP:
    def __init__(self, N, dt, vehicle_model, pedestrian_model, 
                 vehicle_state_dim, vehicle_ctrl_dim,
                 ped_state_dim, ped_ctrl_dim,
                 D_safe):
        """
        Joint QP optimization: all dynamics and cost are assumed linear,
        and the collision avoidance constraint is linearized.
        
        NOTE: The original collision avoidance constraint is
            (x_v[0]-x_p[0])^2 + (x_v[1]-x_p[1])^2 >= D_safe^2.
        Here we linearize it around a nominal point.
        """
        self.N = N
        self.dt = dt
        self.D_safe = D_safe
        
        self.opti = ca.Opti()
        
        # Decision variables.
        self.Xv = self.opti.variable(vehicle_state_dim, N+1)
        self.Uv = self.opti.variable(vehicle_ctrl_dim, N)
        self.Xp = self.opti.variable(ped_state_dim, N+1)
        self.Up = self.opti.variable(ped_ctrl_dim, N)
        
        # Parameters for initial states.
        self.x0_v = self.opti.parameter(vehicle_state_dim, 1)
        self.x0_p = self.opti.parameter(ped_state_dim, 1)
        self.opti.subject_to(self.Xv[:, 0] == self.x0_v)
        self.opti.subject_to(self.Xp[:, 0] == self.x0_p)
        
        # Cost function (quadratic).
        total_cost = 0
        Q_vehicle = ca.DM.eye(vehicle_state_dim)
        R_vehicle = ca.DM.eye(vehicle_ctrl_dim)
        Q_ped = ca.DM.eye(ped_state_dim)
        R_ped = ca.DM.eye(ped_ctrl_dim)
        
        for k in range(N):
            x_v = self.Xv[:, k]
            u_v = self.Uv[:, k]
            x_p = self.Xp[:, k]
            u_p = self.Up[:, k]
            
            total_cost += ca.mtimes([x_v.T, Q_vehicle, x_v]) + ca.mtimes([u_v.T, R_vehicle, u_v])
            total_cost += ca.mtimes([x_p.T, Q_ped, x_p]) + ca.mtimes([u_p.T, R_ped, u_p])
            
            # Linear dynamics constraints (assume vehicle_model and pedestrian_model are linear).
            x_v_next = vehicle_model(x_v, u_v)
            self.opti.subject_to(self.Xv[:, k+1] == x_v_next)
            x_p_next = pedestrian_model(x_p, u_p)
            self.opti.subject_to(self.Xp[:, k+1] == x_p_next)
            
            # Linearized collision avoidance constraint.
            # We linearize f(xv,xp) = (xv[0]-xp[0])^2+(xv[1]-xp[1])^2 - D_safe^2 around nominal points.
            if hasattr(self, "prev_nominal"):
                xv_nom = self.prev_nominal['xv']
                xp_nom = self.prev_nominal['xp']
            else:
                xv_nom = ca.DM([0.0, 0.0, 0.0, 0.0])
                xp_nom = ca.DM([0.0, 0.0, 0.0, 0.0])
            diff_nom = xv_nom[0:2] - xp_nom[0:2]
            f_nom = ca.dot(diff_nom, diff_nom) - D_safe**2
            grad_xv = 2 * (xv_nom[0:2] - xp_nom[0:2])
            grad_xp = -2 * (xv_nom[0:2] - xp_nom[0:2])
            lin_constr = f_nom \
                + ca.dot(grad_xv, x_v[0:2] - xv_nom[0:2]) \
                + ca.dot(grad_xp, x_p[0:2] - xp_nom[0:2])
            self.opti.subject_to(lin_constr >= 0)
            
        # Terminal cost.
        x_v_final = self.Xv[:, N]
        x_p_final = self.Xp[:, N]
        total_cost += ca.mtimes([x_v_final.T, Q_vehicle, x_v_final])
        total_cost += ca.mtimes([x_p_final.T, Q_ped, x_p_final])
        
        self.opti.minimize(total_cost)
        
        # Use a QP solver (e.g., qpoases).
        opts = {"print_time": False, "qpoases": {"printLevel": 0, "max_iter": 1000}}
        self.opti.solver('qpoases', opts)
        self.prev_solution = None
        self.prev_nominal = None

    def do_optimization(self, init_state, target_state=None):
        vehicle_state_dim = self.x0_v.size()[0]
        init_vehicle = init_state[:vehicle_state_dim]
        init_ped = init_state[vehicle_state_dim:]
        self.opti.set_value(self.x0_v, init_vehicle)
        self.opti.set_value(self.x0_p, init_ped)
        
        if self.prev_solution is not None:
            self.opti.set_initial(self.Xv, self.prev_solution['Xv'])
            self.opti.set_initial(self.Uv, self.prev_solution['Uv'])
            self.opti.set_initial(self.Xp, self.prev_solution['Xp'])
            self.opti.set_initial(self.Up, self.prev_solution['Up'])
        sol = self.opti.solve()
        Uv_opt = sol.value(self.Uv)
        Up_opt = sol.value(self.Up)
        self.prev_solution = {
            'Xv': sol.value(self.Xv),
            'Uv': Uv_opt,
            'Xp': sol.value(self.Xp),
            'Up': Up_opt
        }
        # Save nominal points for collision constraint linearization.
        self.prev_nominal = {
            'xv': sol.value(self.Xv)[:, 0],
            'xp': sol.value(self.Xp)[:, 0]
        }
        return Uv_opt, Up_opt

    @property
    def cat_states(self):
        """Return the combined state prediction (vehicle and pedestrian)."""
        if self.prev_solution is not None:
            return np.vstack((self.prev_solution['Xv'], self.prev_solution['Xp']))
        else:
            return np.empty((0,))
