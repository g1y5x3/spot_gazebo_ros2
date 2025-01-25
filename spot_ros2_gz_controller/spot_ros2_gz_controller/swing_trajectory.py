import numpy as np
from pydrake.trajectories import PiecewisePolynomial

from .robot_state import RobotState
from .gait_scheduler import GaitScheduler

class SwingTrajectory():
    def __init__(self, swing_height = 0.1):
        
        self.swing_height = swing_height

        self.current_positions = np.zeros((4,3))
        self.target_positions  = np.zeros((4,3))

        self.K_p = np.zeros((3,3))
        self.K_d = np.zeros((3,3))
        self.tau_ff = 0
        self.p_des = np.zeros((4,3))

        self.foot_state_map = {
            "states": ["stance"] * 4,
            "trajectories" : [None] * 4,
            "transition_time": [0.0] * 4,
            "needs_planning" : [False] * 4
        }

    def update_foot_states(self, gait_schedule: GaitScheduler):
        legs_needing_plans = []
        for leg_idx in range(4):
            current_state = gait_schedule.get_leg_state(leg_idx)
            previous_state = self.foot_state_map["states"][leg_idx]

            if current_state != previous_state:
                self.foot_state_map["states"][leg_idx] = current_state
                self.foot_state_map["transition_time"][leg_idx] = gait_schedule.current_phase

                if current_state == "swing":
                    self.foot_state_map["needs_planning"][leg_idx] = True
                    legs_needing_plans.append(leg_idx)

        return legs_needing_plans

    def update_swingfoot_trajectory(self, robot_state: RobotState, gait_schedule: GaitScheduler):
        # obtain current foot position and velocity, hip position, and CoM velocity

        # body coordinate frame
        foot_pos = robot_state.foot_pos
        foot_vel = robot_state.foot_vel
        hip_pos  = robot_state.hip_pos

        com_vel_des = [1, 0.0, 0.0]

        # world coordinate frame (since they are both obtained from odometry)
        com_pos = robot_state.p
        com_vel = robot_state.p_dot

        H_wb    = robot_state.H_w_base

        legs_for_replanning = self.update_foot_states(gait_schedule)

        # plan new foot placement
        for leg_idx in legs_for_replanning:
            print(leg_idx)

            t_stance, t_swing = gait_schedule.t_stance, gait_schedule.t_swing

            # calculate the desired foot position under the world coordinate frame
            foot_pos_w = np.matmul(H_wb, np.append(foot_pos[:,leg_idx], 1))[:3]
            foot_pos_des = self.foot_planner(t_stance, H_wb, com_pos, com_vel, foot_pos_w, com_vel_des, hip_pos[:,leg_idx])
            
            # generate the trajectory

        # print(f"foot position {foot_position} {np.size(foot_position)}")
        # print(f"foot velocity {foot_velocity} {np.size(foot_velocity)}")
        # print(f"hip position {hip_position} {np.size(hip_position)}")
        # print(f"com velocity {com_vel} {np.size(com_vel)}")

    def foot_planner(self, t_stance, H_wb, p_w, pdot_w, foot_w, pdot_d, hip):
        # convert to world coordinate frame
        # foot_w = np.matmul(H_wb, np.append(foot_position, 1))[:3]
        hip_w = np.matmul(H_wb, np.append(hip, 1))[:3]
        pdot_d_w = np.matmul(H_wb, np.append(pdot_d, 1))[:3]
        print(f"pdot_d {pdot_d_w}")

        # Raibert heuristic
        foot_des = hip_w + (t_stance * pdot_d_w) + np.sqrt(p_w[2]/9.81) * (pdot_w - pdot_d_w)

        # TODO estimate ground level
        foot_des[2] = foot_w[2]
        print(f"current {foot_w}")
        print(f"desired {foot_des}")
        return foot_des

    def generate_swing_trajectory(start_pos, end_pos, t_swing):
        t_breakpoints = np.array([[0.],[t_swing/2], [t_swing]])

        mid_pos = (start_pos + end_pos) /2
        mid_pos[2] = self.swing_height

        foot_pos_breakpoints = np.hstack((
            start_pos.reshape(3,1),
            mid_pos.reshape(3,1),
            end_pos.reshape(3,1),
        ))
        vel_breakpoints = np.zeros((3,3))
        swing_traj = PiecewisePolynomial.CubicHermite(t_breakpoints, foot_pos_breakpoints, vel_breakpoints)
