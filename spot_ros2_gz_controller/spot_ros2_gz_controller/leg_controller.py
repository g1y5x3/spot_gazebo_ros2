import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .robot_state import RobotState
from .gait_scheduler import GaitScheduler
from .swing_trajectory import SwingTrajectory
from .mpc_controller import MPCController

class LegController():
    def __init__(self):
        self.Kp = np.diag([40., 40., 50.])
        self.Kd = np.diag([1., 1., 1.])
        self.joint_names = [
            'front_left_hip_x',  'front_left_hip_y',  'front_left_knee',
            'front_right_hip_x', 'front_right_hip_y', 'front_right_knee',
            'rear_left_hip_x',   'rear_left_hip_y',   'rear_left_knee',
            'rear_right_hip_x',  'rear_right_hip_y',  'rear_right_knee'
        ]

    def update(self,
               trajectory_pub, 
               robot_state: RobotState, gait_schedule: GaitScheduler, 
               swing_traj: SwingTrajectory, mpc_ctrl: MPCController):
        
        torque_cmds = np.zeros(12)

        # swing foot
        J = robot_state.foot_J
        p = robot_state.foot_pos
        v = robot_state.foot_vel
        p_ref = swing_traj.foot_pos_des
        v_ref = swing_traj.foot_vel_des

        # stance foot
        R_bw = robot_state.H_base_w[:3,:3]
        f = mpc_ctrl.f

        for leg_idx in range(4):
            # if gait_schedule.get_leg_state(leg_idx) == "swing":
            #     # TODO Need to unify indexing
            #     tau = J[leg_idx,:].T @ (
            #         self.Kp @ (p_ref[leg_idx,:] - p[:,leg_idx]) + 
            #         self.Kd @ (v_ref[leg_idx,:] - v[:,leg_idx])
            #     )
            #     torque_cmds[3*leg_idx : 3*(leg_idx+1)] = tau
            #     # torque_cmds[3*(leg_idx+2) : 3*(leg_idx+3)] = tau
            #     print(f"torque_cmds {torque_cmds}")
            #     # print(leg_idx)
            #     # print(f"tau {tau}")
            # else:
            if gait_schedule.get_leg_state(leg_idx) == "stance":
                tau = J[leg_idx,:].T @ R_bw @ f[3*leg_idx:3*(leg_idx+1)]
                torque_cmds[3*leg_idx : 3*(leg_idx+1)] = tau

        print(f"torque {torque_cmds}")

        msg = JointTrajectory()
        msg.joint_names = self.joint_names[3:]

        point = JointTrajectoryPoint()
        point.effort = torque_cmds[3:].tolist()
        point.time_from_start.sec = int(0)
        point.time_from_start.nanosec = int(0)

        msg.points = [point]
        trajectory_pub.publish(msg)
