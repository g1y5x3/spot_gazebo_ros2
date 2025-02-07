import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.math import RollPitchYaw, RigidTransform
from pydrake.common.eigen_geometry import Quaternion


def quat_to_euler(q):
    quat = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
    rpy = RollPitchYaw(quat.rotation())

    return np.array([rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle()])
    
def pose_to_homogeneous(p, q):
    quat = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
    translation = [p.x, p.y, p.z]
    T = RigidTransform(quaternion=quat, p=translation)

    return T.GetAsMatrix4()

def invert_homogeneous(H):
    R = H[:3, :3]
    t = H[:3, 3]

    H_inv = np.eye(4)
    H_inv[:3, :3] = R.T
    H_inv[:3, 3] = -R.T @ t

    return H_inv


class RobotState:
    def __init__(self, model_sdf: str):

        # Actuated joint configuration for quadruped robot:
        # joint_names:    List of 12 actuated joints, organized by leg (front left, front right, rear left, rear right)
        #                 Each leg has 3 joints: hip_x (abduction/adduction), hip_y (flexion/extension), knee (flexion/extension)
        # joint_position: Array of joint angles [rad], indexed in same order as joint_names
        # joint_velocity: Array of joint angular velocities [rad/s], indexed in same order as joint_names
        self.joint_names = [
            'front_left_hip_x',  'front_left_hip_y',  'front_left_knee',
            'front_right_hip_x', 'front_right_hip_y', 'front_right_knee',
            'rear_left_hip_x',   'rear_left_hip_y',   'rear_left_knee',
            'rear_right_hip_x',  'rear_right_hip_y',  'rear_right_knee'
        ]
        self.joint_position = np.zeros(12)
        self.joint_velocity = np.zeros(12)

        # hip position, foot position, velocities, and jacobians under the robot base frame
        self.hip_pos  = np.zeros((3,4))
        self.foot_pos = np.zeros((3,4))
        self.foot_vel = np.zeros((3,4))
        self.foot_J = np.zeros((4,3,3))

        # State variables for rigid body dynamics:
        # p:         Position vector (x, y, z) of the center of mass in world coordinates [m]
        # p_dot:     Linear velocity vector of the center of mass [m/s]
        # theta:     Euler angles (roll, pitch, yaw) representing orientation [rad]
        # theta_dot: Angular velocity vector in world frame [rad/s]
        self.p = np.zeros(3)
        self.p_dot = np.zeros(3)
        self.theta = np.zeros(3)
        self.theta_dot = np.zeros(3)

        # Homogeneous transformation matrices (4x4) for coordinate frame conversions:
        # H_world_base: Transforms coordinates from base frame to world frame
        # H_base_world: Transforms coordinates from world frame to base frame (inverse of H_world_base)
        self.H_world_base = np.zeros((4,4))
        self.H_base_world = np.zeros((4,4))

        # Kinematic state variables for quadruped legs:
        # p_hip_base:   Hip positions in base frame [m], array shape (4,3) for [FL, FR, RL, RR]
        # p_hip_world:  Hip positions in world frame [m]
        # p_foot_base:  Foot positions in base frame [m]
        # p_foot_world: Foot positions in world frame [m]
        # v_foot_base:  Foot velocities in base frame [m/s]
        # v_foot_world: Foot velocities in world frame [m/s]
        # J_foot_base:  Foot Jacobian matrices in base frame, shape (4,3,3) where each (3,3) maps
        #               joint velocities to foot velocities for one leg [dimensionless]
        self.p_hip_base  = np.zeros((4,3))
        self.p_hip_world = np.zeros((4,3))
        self.p_foot_base  = np.zeros((4,3))
        self.p_foot_world = np.zeros((4,3))
        self.v_foot_base  = np.zeros((4,3))
        self.v_foot_world = np.zeros((4,3))
        self.J_foot_base = np.zeros((4,3,3))

        # Inertial properties of the robot base:
        # I_base: 3x3 inertia tensor/matrix about the center of mass [kg⋅m²]
        # mass:   Total mass of the robot base [kg]
        # TODO: Load these params from a config file instead of hardcode them
        self.I_base = np.array([[ 0.287877,   0.0014834,   -0.0347842],
                                [ 0.0014834,  1.31868,     -0.000519074],
                                [-0.0347842, -0.000519074,  1.18915]])

        self.mass = 38.492

        # using drake library for kinematics and dynamics calculation
        # https://github.com/RobotLocomotion/drake
        # TODO: derive from scratch
        self.plant = MultibodyPlant(time_step=0.0)
        Parser(self.plant).AddModels(model_sdf)
        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()
        print("Pydrake initialized!")
        print(f"number of positions: {self.plant.num_positions()}")
        print(f"position names: {self.plant.GetPositionNames()}")

    def update(self, jointstate_msg: JointState, odom_msg: Odometry):
        self.update_pose(odom_msg)
        self.update_joints(jointstate_msg)
        self.update_feet()

    def update_pose(self, msg: Odometry):
        p = msg.pose.pose.position
        self.p = np.array([p.x, p.y, p.z])
        q = msg.pose.pose.orientation
        self.theta = quat_to_euler(q)

        pdot = msg.twist.twist.linear
        self.p_dot = np.array([pdot.x, pdot.y, pdot.z])
        qdot = msg.twist.twist.angular
        self.theta_dot = np.array([qdot.x, qdot.y, qdot.z])

        self.H_world_base = pose_to_homogeneous(p, q)
        self.H_base_world = invert_homogeneous(self.H_world_base)

    def update_joints(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.joint_position[idx] = msg.position[i]
                self.joint_velocity[idx] = msg.velocity[i]

        current_positions = self.plant.GetPositions(self.context)
        current_velocities = self.plant.GetVelocities(self.context)

        current_positions[7:19] = self.joint_position
        current_velocities[6:18] = self.joint_velocity

        self.plant.SetPositions(self.context, current_positions)
        self.plant.SetVelocities(self.context, current_velocities)

    def update_feet(self):
        frame_base = self.plant.GetFrameByName("base_link")

        frames_foot = ["front_left_ee", "front_right_ee",
                       "rear_left_ee",  "rear_right_ee"]

        frames_hip  = ["front_left_upper_leg", "front_right_upper_leg",
                       "rear_left_upper_leg",  "rear_right_upper_leg"]
 
        joint_velocity = np.zeros(19)
        joint_velocity[7:19] = self.joint_velocity

        for i in range(4):
            frame_foot = self.plant.GetFrameByName(frames_foot[i])
            frame_hip  = self.plant.GetFrameByName(frames_hip[i])

            foot_position = self.plant.CalcRelativeTransform(
                context=self.context,
                frame_B=frame_foot,
                frame_A=frame_base
            )
            # self.foot_pos[:,i] = foot_position.translation()
            self.p_foot_base[i,:] = foot_position.translation()
            self.p_foot_world[i,:] = self.H_world_base[:3,:3] @ foot_position.translation() + self.H_world_base[:3,3]

            hip_position = self.plant.CalcRelativeTransform(
                context=self.context,
                frame_B=frame_hip,
                frame_A=frame_base
            )
            # self.hip_pos[:,i] = hip_position.translation()
            self.p_hip_base[i,:] = hip_position.translation()
            self.p_hip_world[i,:] = self.H_world_base[:3,:3] @ hip_position.translation() + self.H_world_base[:3,3]

            foot_jacobian = self.plant.CalcJacobianTranslationalVelocity(
                context=self.context,
                with_respect_to=JacobianWrtVariable.kQDot,
                frame_B=frame_foot,
                p_BoBi_B=np.zeros(3),
                frame_A=frame_base,
                frame_E=frame_base
            )

            # skip the first 7 elements because they were for [x, y, z, wx, wy, wz, w]
            # self.foot_J[i] = foot_jacobian[:,i*3+7:i*3+10]
            self.J_foot_base[i,:] = foot_jacobian[:,i*3+7:i*3+10]

            # self.foot_vel[:,i] = foot_jacobian @ joint_velocity
            self.v_foot_base[i,:] = J_foot_base[i,:] @ self.joint_velocity[i*3:i*3+3]
