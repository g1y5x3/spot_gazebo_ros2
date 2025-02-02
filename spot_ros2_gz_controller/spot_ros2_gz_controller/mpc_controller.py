import time
import numpy as np
from scipy.linalg import expm
from pydrake.all import MathematicalProgram, Solve

from .robot_state import RobotState
from .gait_scheduler import GaitScheduler


def r_cross(r):
    """
    Compute [r]×

    Args:
        r: 3D position vector
    Returns:
        3x3 matrix result
    """
    return np.array([[    0, -r[2],  r[1]],
                     [ r[2],     0, -r[0]],
                     [-r[1],  r[0],     0]])


class MPCController:
    def __init__(self, robot_state: RobotState, gait_cycle=0.5, horizon=16):
        self.dt = gait_cycle/horizon
        self.horizon = horizon
        self.fz_max = 666    # (N) maximum normal force
        self.fz_min = 10     # (N) minimum normal force
        self.mu = 0.6

        # quadratic programming weights
        # r, p, y, x, y, z, wx, wy, wz, vx, vy, vz, g
        self.L = np.kron(np.identity(self.horizon),
                         np.diag([5., 5., 10., 10., 10., 50., 0.01, 0.01, 0.2, 0.2, 0.2, 0.2, 0.]))

        self.K = np.kron(np.identity(self.horizon),
                         np.diag([1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5]))

        # initialized the desired trajectory as the robot initial pose
        self.mass = robot_state.mass
        self.p_x_des = robot_state.p[0]
        self.p_y_des = robot_state.p[1]
        self.p_z_des = robot_state.p[2] # assume z position to be constant
        self.roll_des  = robot_state.theta[0]
        self.pitch_des = robot_state.theta[1]
        self.yaw_des   = robot_state.theta[2]
        print(f"p_x_des {self.p_x_des}")
        print(f"p_y_des {self.p_y_des}")

        self.f = np.zeros(12)

    # TODO reduce the MPC calculation frequency
    def udpate_control(self, robot_state: RobotState, gait_schedule: GaitScheduler):
        # TODO: THIS SHOULD BE IN TRAJECTORY CLASS NOT HERE
        # also read com vel from control inputs
        com_vel_des = [1, 0.0, 0.0]
        com_vel_des_w = np.matmul(robot_state.H_w_base[:3,:3], com_vel_des)

        # Obtain the current state
        x = self.get_state_vec(robot_state)
        yaw      = robot_state.theta[2]
        foot_pos = robot_state.foot_pos
        # print(f"current state vecotr {x}")

        # Generate reference trajectory for only xy position and yaw
        # NOTE this part doesn't feel like they blong here
        # TODO: self.yaw += dt_control ...
        self.p_x_des += 0.001 * com_vel_des_w[0]
        self.p_y_des += 0.001 * com_vel_des_w[1]

        # # MPC solver (NOTE: could happen less frequent)
        # start = time.perf_counter()
        # x_ref = self.generate_reference_trajectory(com_vel_des_w)
        # Ac, Bc = self.construct_state_space_model(yaw, foot_pos, robot_state.I)
        # Ad, Bd = self.linear_discretize(Ac, Bc)
        # H, g = self.QP_formulation(Ad, Bd, x, x_ref)
        # C, C_lb, C_ub = self.QP_constraints(gait_schedule.contact_schedule)
        # U = self.solve_QP(H, g, C, C_lb, C_ub)
        # print(f"MPC solve time: {time.perf_counter() - start:.5f}s")

        # # "The desired ground reaction forces are then the first 3n elements of U"
        # self.f = U[:12]

    def get_state_vec(self, robot_state: RobotState):
        # - θ (roll, pitch, yaw angles) [3]
        # - p (position) [3]
        # - ω (angular velocity) [3]
        # - ṗ (linear velocity) [3]
        # - g (gravity constant) [1]

        state = np.zeros(13)
        state[0:3]  = robot_state.theta
        state[3:6]  = robot_state.p
        state[6:9]  = robot_state.omega
        state[9:12] = robot_state.p_dot
        state[12]   = -9.81

        return state

    # TODO Add yaw rate
    def generate_reference_trajectory(self, com_vel_des):
        x_ref = np.zeros(13 * self.horizon)
        x_ref[0::13] = self.roll_des
        x_ref[1::13] = self.pitch_des
        x_ref[2::13] = self.yaw_des
        x_ref[3] = self.p_x_des
        x_ref[4] = self.p_y_des
        for i in range(1, self.horizon):
            x_ref[3+13*i] = x_ref[3+13*(i-1)] + self.dt * com_vel_des[0]
            x_ref[4+13*i] = x_ref[4+13*(i-1)] + self.dt * com_vel_des[1]
        # use the current robot height which is constant
        x_ref[5::13] = self.p_z_des
        x_ref[9::13] = com_vel_des[0]
        x_ref[10::13] = com_vel_des[1]
        x_ref[12::13] = -9.81

        return x_ref

    def construct_state_space_model(self, yaw, foot_pos, I):
        # Construct state space model (Eq. 16, 17)
        Ac = np.zeros((13, 13))
        Bc = np.zeros((13, 3*4))

        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw),  np.cos(yaw), 0],
                       [          0,            0, 1]])
        I_w = Rz @ I @ Rz.T

        Ac[0:3, 6:9]  = Rz.T
        Ac[3:6, 9:12] = np.identity(3)
        Ac[11, 12]    = 1.0

        # \hat{I}^-1[r_i]x
        for i in range(4):
            Bc[6:9,  3*i:3*i+3] = np.linalg.inv(I_w) @ r_cross(foot_pos[:,i])
            Bc[9:12, 3*i:3*i+3] = np.identity(3) / self.mass

        return Ac, Bc

    def linear_discretize(self, Ac, Bc):
        # Discrete time dynamics (Eq. 25)
        # square_matrix = [[Ac (13*13), Bc (13*12)],
        #                  [0  (12*13), 0  (12*12)]] * dt (25*25)
        square_matrix = np.zeros((25, 25))
        square_matrix[0:13, 0:13] = Ac * self.dt
        square_matrix[0:13, 13:25] = Bc * self.dt
        # print(f"square matrix {square_matrix.shape}")

        #     [[Ac, Bc],          [[Ad, Bd],
        # exp( [ 0,  0]] * dt) =   [ 0,  I]]
        matrix_exponential = expm(square_matrix)
        Ad = matrix_exponential[0:13, 0:13]
        Bd = matrix_exponential[0:13, 13:25]

        return Ad, Bd

    def QP_formulation(self, Ad, Bd, x, x_ref):
        # QP formulation
        # Equation
        # min_U (1/2)UᵀHU + Uᵀg            (Eq. 29)
        # where H = 2(B_qpᵀ L B_qp + K)    (Eq. 31)
        #       g = 2B_qpᵀ L(A_qp x₀ - y)  (Eq. 32)

        # Precompute powers of Ad: [I, Ad, Ad^2, ..., Ad^horizon]
        power_of_A = [np.identity(13)]
        for i in range(self.horizon):
            power_of_A.append(power_of_A[i] @ Ad)

        A_qp = np.vstack(power_of_A[1:self.horizon+1])

        B_qp =  np.zeros((13 * self.horizon, 12 * self.horizon))
        for i in range(self.horizon):
            for j  in range(self.horizon):
                if i >= j:
                    B_qp[13*i:13*(i+1), 12*j:12*(j+1)] = power_of_A[i-j] @ Bd

        H = 2 * (B_qp.T @ self.L @ B_qp + self.K)
        g = 2 * B_qp.T @ self.L @ (A_qp @ x - x_ref)

        return H, g

    def QP_constraints(self, contact_schedule):
        # QP constraints
        constraint_coef_matrix = np.array([
            [ 1,  0, -self.mu], #  f_x <= mu f_z
            [-1,  0, -self.mu], # -f_x <= mu f_z
            [ 0,  1, -self.mu], #  f_y <= mu f_z
            [ 0, -1, -self.mu], # -f_y <= mu f_z
            [ 0,  0,        1], # f_min <= f_z <= f_max
        ])
        C = np.kron(np.identity(4*self.horizon), constraint_coef_matrix)

        C_lb = np.zeros(4*5*self.horizon)
        C_ub = np.zeros(4*5*self.horizon)
        for i in range(self.horizon):
            for j in range(4):
                idx = i * 4 + j
                C_lb[5*idx : 5*idx+4] = -np.inf
                C_ub[5*idx : 5*idx+4] = 0
                C_ub[5*idx+4] = contact_schedule[j, i] * self.fz_max
        # print(C, C_lb, C_ub)

        return C, C_lb, C_ub

    def solve_QP(self, H, g, C, C_lb, C_ub):
        qp_problem = MathematicalProgram()
        U = qp_problem.NewContinuousVariables(12 * self.horizon, 'U')

        qp_problem.AddQuadraticCost(H, g, U)
        qp_problem.AddLinearConstraint(C, C_lb, C_ub, U)

        result = Solve(qp_problem)

        return result.GetSolution(U)
