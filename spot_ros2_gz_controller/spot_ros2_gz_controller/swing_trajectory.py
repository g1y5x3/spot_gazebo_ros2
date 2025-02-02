import numpy as np
from pydrake.trajectories import PiecewisePolynomial

from .robot_state import RobotState
from .gait_scheduler import GaitScheduler


class SwingTrajectory():
    def __init__(self, swing_height = 0.1):
        
        self.swing_height = swing_height

        self.foot_pos_des = np.zeros((4,3))
        self.foot_vel_des = np.zeros((4,3))

        self.tau_ff = 0

        self.foot_state_map = {
            "states": ["stance"] * 4,
            "trajectories" : [None] * 4,
            "transition_time": [0.0] * 4,
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
                    legs_needing_plans.append(leg_idx)

        return legs_needing_plans

    def update_swingfoot_trajectory(self, robot_state: RobotState, gait_schedule: GaitScheduler):
        # TODO: read com_vel from the control inputs
        com_vel_des = [1, 0.0, 0.0] # desired CoM velocity in body frame

        foot_pos = robot_state.foot_pos # current foot positions in body frame
        # foot_vel = robot_state.foot_vel # current foot velocities in body frame
        hip_pos  = robot_state.hip_pos  # current hip positions in body frame
        com_pos_w = robot_state.p       # CoM position in world frame
        com_vel_w = robot_state.p_dot   # CoM velocity in world frame

        H_wb = robot_state.H_w_base # transformation matrix from body to world frame 
        H_bw = robot_state.H_base_w # transformation matrix from world to body frame

        # update current foot positions and velocities
        # self.foot_pos = foot_pos
        # self.foot_vel = foot_vel

        # update foot states and get legs that need replanning
        legs_for_replanning = self.update_foot_states(gait_schedule)

        # plan new foot placement for legs that need planning
        for leg_idx in legs_for_replanning:
            # print(leg_idx)
            t_stance, t_swing = gait_schedule.t_stance, gait_schedule.t_swing

            # calculate the desired foot position in world frame
            foot_pos_w = np.matmul(H_wb, np.append(foot_pos[:,leg_idx], 1))[:3]
            foot_pos_des_placement_w = self.foot_planner(t_stance, H_wb, com_pos_w, com_vel_w, foot_pos_w, com_vel_des, hip_pos[:,leg_idx])
            
            # generate the swing trajectory for this leg
            self.foot_state_map["trajectories"][leg_idx] = self.generate_swing_trajectory(foot_pos_w, foot_pos_des_placement_w, t_swing)

        # update the desired foot position and velocity based on current phase and planned trajectory
        for leg_idx in range(4):
            if self.foot_state_map["states"][leg_idx] == "swing":
                # print(leg_idx)
                phase_time = gait_schedule.current_phase - self.foot_state_map["transition_time"][leg_idx]
                swing_traj = self.foot_state_map["trajectories"][leg_idx]
                if swing_traj is not None:
                    # both were based off pydrake PiecewisePolynomial API
                    foot_pos_des_swing = swing_traj.value(phase_time).flatten()
                    foot_vel_des_swing = swing_traj.derivative(1).value(phase_time).flatten()

                    # transform desired foot position and velocity back to body frame
                    foot_pos_des = np.matmul(H_bw, np.append(foot_pos_des_swing, 1))[:3]
                    foot_vel_des = np.matmul(H_bw[:3, :3], foot_vel_des_swing)
                    
                    # update the desired foot position and velocity
                    self.foot_pos_des[leg_idx, :] = foot_pos_des
                    self.foot_vel_des[leg_idx, :] = foot_vel_des

    def foot_planner(self, t_stance, H_wb, p_w, pdot_w, foot_w, pdot_d, hip):
        # convert to world coordinate frame
        # foot_w = np.matmul(H_wb, np.append(foot_position, 1))[:3]
        hip_w = np.matmul(H_wb, np.append(hip, 1))[:3]
        pdot_d_w = np.matmul(H_wb[:3,:3], pdot_d)
        # print(f"pdot_d {pdot_d_w}")

        # Raibert heuristic
        foot_des = hip_w + (t_stance*pdot_d_w/2) + np.sqrt(p_w[2]/9.81) * (pdot_w - pdot_d_w)

        # TODO estimate ground level
        foot_des[2] = foot_w[2]
        # print(f"current {foot_w}")
        # print(f"desired {foot_des}")
        return foot_des

    def generate_swing_trajectory(self, start_pos, end_pos, t_swing):
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

        return swing_traj

    # def generate_reference_trajectory
