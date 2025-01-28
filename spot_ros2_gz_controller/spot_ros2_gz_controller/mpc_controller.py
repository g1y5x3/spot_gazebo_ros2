import numpy as np

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
    def __init__(self, f_min=0, f_max=1):
        self.f_max = f_max  # minimum normal force
        self.f_min = f_min  # maximum normal force
        self.current_state = np.zeros(13)

    def udpate_control(self, robot_state: RobotState, gait_schedule: GaitScheduler):
        current_state = self.get_state_vec(robot_state)
        yaw = robot_state.theta[2]
        foot_pos = robot_state.foot_pos
        mass = robot_state.mass
        print(current_state)

        # TODO reduce the MPC calculation frequency
        # \dot{x} = A_{c}x + B_{c}u and equation 16
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

