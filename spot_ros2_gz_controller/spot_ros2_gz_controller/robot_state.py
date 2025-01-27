import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from pydrake.multibody.tree import JointIndex, JacobianWrtVariable
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.math import RollPitchYaw, RigidTransform
from pydrake.common.eigen_geometry import Quaternion

def invert_homogeneous_transform(H):
    # Extract the rotation matrix and translation vector
    R = H[:3, :3]  # 3x3 rotation matrix
    t = H[:3, 3]   # 3x1 translation vector

    # Compute the inverse
    R_inv = R.T  # Transpose of R is its inverse
    t_inv = -R_inv @ t  # New translation vector

    # Construct the inverse matrix
    H_inv = np.eye(4)  # Initialize as identity matrix
    H_inv[:3, :3] = R_inv
    H_inv[:3, 3] = t_inv

    return H_inv

class RobotState:
    def __init__(self, model_sdf: str):

        # fixed order of the joints
        self.joint_names = [
            'front_left_hip_x',  'front_left_hip_y',  'front_left_knee',
            'front_right_hip_x', 'front_right_hip_y', 'front_right_knee',
            'rear_left_hip_x',   'rear_left_hip_y',   'rear_left_knee',
            'rear_right_hip_x',  'rear_right_hip_y',  'rear_right_knee'
        ]

        # joint states (following orders in joint_names)
        self.joint_position = np.zeros(12)
        self.joint_velocity = np.zeros(12)

        # hip position, foot position, velocities, and jacobians under the robot base frame
        self.hip_pos  = np.zeros((3,4))
        self.foot_pos = np.zeros((3,4))
        self.foot_vel = np.zeros((3,4))
        self.foot_J = np.zeros((4,3,3))

        # position
        self.p = np.zeros(3)
        self.p_dot = np.zeros(3)    # linear velocity

        # orientation
        self.theta = np.zeros(3)
        self.omega = np.zeros(3)    # angular velocity

        # transformation between the body frame and world coordinate frame
        self.H_w_base = np.zeros((4,4))
        self.H_base_w = np.zeros((4,4))

        # using drake library for kinematics and dynamics calculation
        # https://github.com/RobotLocomotion/drake
        self.plant = MultibodyPlant(time_step=0.0)
        Parser(self.plant).AddModels(model_sdf)
        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()

        current_positions_names = self.plant.GetPositionNames()
        current_positions = self.plant.GetPositions(self.context)
        print(f"Number of positions: {self.plant.num_positions()}")
        print(current_positions_names[7:19])
        print(current_positions[7:19])

        current_velocitiy_names = self.plant.GetVelocityNames()
        current_velocities_names = self.plant.GetVelocities(self.context)
        # print(f"Number of velocities: {self.plant.num_velocities()}")
        # print(current_velocitiy_names[6:18])
        # print(current_velocities_names[6:18])

    def update_joints(self, msg: JointState):
        # print(msg.name)
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                # print(idx)
                self.joint_position[idx] = msg.position[i]
                self.joint_velocity[idx] = msg.velocity[i]

        # drake context return all positions that it tracks internally
        current_positions = self.plant.GetPositions(self.context)
        current_velocities = self.plant.GetVelocities(self.context)

        current_positions[7:19] = self.joint_position
        current_velocities[6:18] = self.joint_velocity

        # set the updated states back to context
        self.plant.SetPositions(self.context, current_positions)
        self.plant.SetVelocities(self.context, current_velocities)

    # TODO: estimate the state based on sensors inputs to replace ground truth
    # provided by gazebo
    def update_pose(self, msg: Odometry):
        # position and pose velocity
        p = msg.pose.pose.position
        pdot = msg.twist.twist.linear

        self.p = np.array([p.x, p.y, p.z])
        self.p_dot = np.array([pdot.x, pdot.y, pdot.z])

        # orientation and angular velocity
        q = msg.pose.pose.orientation
        qdot = msg.twist.twist.angular

        self.theta = self.quat_to_euler(q)
        self.omega = np.array([qdot.x, qdot.y, qdot.z])

        # construct a homogeneous transformation matrix from body to world
        self.H_w_base = self.post_to_homogeneous(p, q)
        self.H_base_w = invert_homogeneous_transform(self.H_w_base)

    def quat_to_euler(self, q):
        quat = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
        rpy = RollPitchYaw(quat.rotation())

        return np.array([rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle()])
    
    def post_to_homogeneous(self, p, q):
        quat = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
        translation = [p.x, p.y, p.z]
        T = RigidTransform(quaternion=quat, p=translation)

        return T.GetAsMatrix4()

    def update_feet(self):
        # TODO: make this part better
        joint_velocity = np.zeros(19)
        joint_velocity[7:19] = self.joint_velocity

        base_frame = self.plant.GetFrameByName("base_link")

        foot_frames = ["front_left_ee", "front_right_ee",
                       "rear_left_ee",  "rear_right_ee"]

        hip_frames  = ["front_left_hip", "front_right_hip",
                       "rear_left_hip",  "rear_right_hip"]

        # B_p_i, B_v_i
        for i in range(4):
            foot_frame = self.plant.GetFrameByName(foot_frames[i])

            hip_frame = self.plant.GetFrameByName(hip_frames[i])

            # foot position B_p_i
            foot_position = self.plant.CalcRelativeTransform(
                context=self.context,
                frame_B=foot_frame,
                frame_A=base_frame
            )
            self.foot_pos[:,i] = foot_position.translation()

            # hip position
            hip_position = self.plant.CalcRelativeTransform(
                context=self.context,
                frame_B=hip_frame,
                frame_A=base_frame
            )
            self.hip_pos[:,i] = hip_position.translation()

            # foot jacobian J_i
            foot_jacobian = self.plant.CalcJacobianTranslationalVelocity(
                context=self.context,
                with_respect_to=JacobianWrtVariable.kQDot,
                frame_B=foot_frame,
                p_BoBi_B=np.zeros(3),
                frame_A=base_frame,
                frame_E=base_frame
            )
            self.foot_J[i] = foot_jacobian[:,i*3+7:i*3+10]

            # foot velocity B_v_i
            self.foot_vel[:,i] = foot_jacobian @ joint_velocity

    # update the sensory readings, all 4 foot positions, jacobians, bias
    def update(self, jointstate_msg: JointState, odom_msg: Odometry):
        if jointstate_msg is not None and odom_msg is not None:
            self.update_pose(odom_msg)
            self.update_joints(jointstate_msg)
            self.update_feet()

    def get_state_vec(self):
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
        state = np.zeros(13, dtype=np.float32)

        state[0:3] = self.theta  # [roll, pitch, yaw]
        state[3:6] = self.p
        state[6:9] = self.omega
        state[9:12] = self.p_dot

        # Add gravity
        state[12] = -9.81

        return state

    def __str__(self):
        output = "===== Robot State ===== \n"
        # Group joints by leg
        fl = f"FL: [{self.joint_position[0]:.3f}, {self.joint_position[1]:.3f},  {self.joint_position[2]:.3f}]"
        fr = f"FR: [{self.joint_position[3]:.3f}, {self.joint_position[4]:.3f},  {self.joint_position[5]:.3f}]"
        rl = f"RL: [{self.joint_position[6]:.3f}, {self.joint_position[7]:.3f},  {self.joint_position[8]:.3f}]"
        rr = f"RR: [{self.joint_position[9]:.3f}, {self.joint_position[10]:.3f}, {self.joint_position[11]:.3f}]"

        output += "joint positions (q):\n"
        output += f"{fl}\n{fr}\n{rl}\n{rr}\n"

        # Same format for velocities
        fl_v = f"FL: [{self.joint_velocity[0]:.3f}, {self.joint_velocity[1]:.3f},  {self.joint_velocity[2]:.3f}]"
        fr_v = f"FR: [{self.joint_velocity[3]:.3f}, {self.joint_velocity[4]:.3f},  {self.joint_velocity[5]:.3f}]"
        rl_v = f"RL: [{self.joint_velocity[6]:.3f}, {self.joint_velocity[7]:.3f},  {self.joint_velocity[8]:.3f}]"
        rr_v = f"RR: [{self.joint_velocity[9]:.3f}, {self.joint_velocity[10]:.3f}, {self.joint_velocity[11]:.3f}]"

        output += "\njoint velocities (q_dot):\n"
        output += f"{fl_v}\n{fr_v}\n{rl_v}\n{rr_v}\n"

        # Position and Velocity
        output += "\nposition (p):\n"
        output += f"[{self.p[0]:.3f}, {self.p[1]:.3f}, {self.p[2]:.3f}]\n"

        output += "\nvelocity (p_dot):\n"
        output += f"[{self.p_dot[0]:.3f}, {self.p_dot[1]:.3f}, {self.p_dot[2]:.3f}]\n"

        # Orientation and Angular Velocity
        output += "\norientation (theta):\n"
        output += f"[{self.theta[0]:.3f}, {self.theta[1]:.3f}, {self.theta[2]:.3f}]\n"

        output += "\nangular velocity (omega):\n"
        output += f"[{self.omega[0]:.3f}, {self.omega[1]:.3f}, {self.omega[2]:.3f}]\n"

        return output
