import numpy as np

from .robot_state import RobotState
from .gait_scheduler import GaitScheduler

"""
Carlo, Jared Di, Wensing, Patrick M., Katz, Benjamin, Bledt, Gerardo
and Kim, Sangbae. 2018. "Dynamic Locomotion in the MIT Cheetah
3 Through Convex Model-Predictive Control." IEEE International
Conference on Intelligent Robots and Systems.
"""

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
    def __init__(self, robot_state: RobotState, f_min=0, f_max=1):
        self.f_max = f_max  # minimum normal force
        self.f_min = f_min  # maximum normal force
        self.dt = 0.001
        self.horizon = 16

        # initialized the desired trajectory as the robot initial pose
        self.p_x_des = robot_state.p[0]
        self.p_y_des = robot_state.p[1]
        self.p_z_des = robot_state.p[2] # assume z position to be constant
        self.roll_des  = robot_state.theta[0]
        self.pitch_des = robot_state.theta[1]
        self.yaw_des   = robot_state.theta[2]

    # TODO reduce the MPC calculation frequency
    def udpate_control(self, robot_state: RobotState, gait_schedule: GaitScheduler):
        # TODO: THIS SHOULD BE IN TRAJECTORY CLASS NOT HERE
        # also read com vel from control inputs
        com_vel_des = [1, 0.0, 0.0]
        com_vel_des_w = np.matmul(robot_state.H_w_base[:3,:3], com_vel_des)

        # obtain the current state
        x = self.get_state_vec(robot_state)
        yaw = robot_state.theta[2]
        foot_pos = robot_state.foot_pos
        mass = robot_state.mass
        print(f"current state vecotr {x}")

        # generate reference trajectory for only xy position and yaw
        self.p_x_des += self.dt * com_vel_des_w[0]
        self.p_y_des += self.dt * com_vel_des_w[1]
        # TODO: self.yaw += dt_control ...
        x_ref = np.zeros(13 * self.horizon)
        x_ref[0::13] = self.roll_des
        x_ref[1::13] = self.pitch_des
        x_ref[2::13] = self.yaw_des
        x_ref[3] = self.p_x_des
        x_ref[4] = self.p_y_des
        x_ref[5::13] = self.p_z_des 
        for i in range(1, self.horizon):
            x_ref[3+13*i] = x_ref[3+13*(i-1)] + self.dt * com_vel_des_w[0]
            x_ref[4+13*i] = x_ref[4+13*(i-1)] + self.dt * com_vel_des_w[1]


        Ac = np.zeros((13, 13))
        Bc = np.zeros((13, 3*4))

        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw),  np.cos(yaw), 0],
                       [          0,            0, 1]])

        I_w = Rz @ robot_state.I @ Rz.T

        Ac[0:3, 6:9] = Rz.T
        Ac[3:6, 9:12] = np.identity(3)
        Ac[11, 12] = 1.0

        # I^-1[r_i]x
        for i in range(4):
            Bc[6:9, 3*i:3*i+3] = np.linalg.inv(I_w) @ r_cross(foot_pos[:,i])
            Bc[9:12, 3*i:3*i+3] = np.identity(3) / mass

    # TODO Add yaw rate
    def generate_reference_trajectory(self, com_vel_des):
        # TODO error handling
        pass

    def get_state_vec(self, robot_state: RobotState):
        """
        Returns the full 13-dimensional state vector required by the MPC controller.

        The state vector consists of:
        - θ (roll, pitch, yaw angles) [3]
        - p (position) [3]
        - ω (angular velocity) [3]
        - ṗ (linear velocity) [3]
        - g (gravity constant) [1]

        Returns:
            np.ndarray: 13-element state vector [θ, p, ω, ṗ, g]
        """
        state = np.zeros(13)

        state[0:3] = robot_state.theta
        state[3:6] = robot_state.p
        state[6:9] = robot_state.omega
        state[9:12] = robot_state.p_dot
        state[12] = -9.81

        return state

    # dynamic constraints: \dot{x} = A_{c}x + B_{c}u
    # def construct_dynamics(self, x, u):

