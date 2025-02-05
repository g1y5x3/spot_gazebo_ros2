import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from pydrake.multibody.tree import JointIndex, JacobianWrtVariable
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.math import RollPitchYaw, RigidTransform
from pydrake.common.eigen_geometry import Quaternion

def quat_to_euler(q):
    quat = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
    rpy = RollPitchYaw(quat.rotation())

    return np.array([rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle()])
    
def pose_to_homogeneous(p, q):
    # p - position, q - orientation
    quat = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
    translation = [p.x, p.y, p.z]
    T = RigidTransform(quaternion=quat, p=translation)

    return T.GetAsMatrix4()

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

        # TODO: Load these params from file instead of hardcode them
        self.I = np.array([[ 0.287877,   0.0014834,   -0.0347842],
                           [ 0.0014834,  1.31868,     -0.000519074],
                           [-0.0347842, -0.000519074,  1.18915]])

        self.mass = 38.492

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

    def update_pose(self, msg: Odometry):
        # position and pose velocity
        p = msg.pose.pose.position
        self.p = np.array([p.x, p.y, p.z])
        q = msg.pose.pose.orientation
        self.theta = quat_to_euler(q)

        pdot = msg.twist.twist.linear
        self.p_dot = np.array([pdot.x, pdot.y, pdot.z])
        qdot = msg.twist.twist.angular
        self.omega = np.array([qdot.x, qdot.y, qdot.z])

        # construct a homogeneous transformation matrix from body to world
        self.H_w_base = pose_to_homogeneous(p, q)
        self.H_base_w = invert_homogeneous_transform(self.H_w_base)

    def update_joints(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.joint_position[idx] = msg.position[i]
                self.joint_velocity[idx] = msg.velocity[i]

        # drake context return all positions that it tracks internally
        current_positions = self.plant.GetPositions(self.context)
        current_velocities = self.plant.GetVelocities(self.context)

        current_positions[7:19] = self.joint_position
        current_velocities[6:18] = self.joint_velocity

        # set the updated states back to context
        # print(f"joint position {self.joint_position}")
        # print(f"joint velocity {self.joint_velocity}")

        self.plant.SetPositions(self.context, current_positions)
        self.plant.SetVelocities(self.context, current_velocities)

    def update_feet(self):
        # TODO: make this part better
        joint_velocity = np.zeros(19)
        joint_velocity[7:19] = self.joint_velocity

        base_frame = self.plant.GetFrameByName("base_link")

        foot_frames = ["front_left_ee", "front_right_ee",
                       "rear_left_ee",  "rear_right_ee"]

        hip_frames  = ["front_left_upper_leg", "front_right_upper_leg",
                       "rear_left_upper_leg",  "rear_right_upper_leg"]

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

            # print(foot_jacobian.shape)
            # print(foot_jacobian)
            # print(foot_jacobian[:,i*3+7:i*3+10])

            # the first 7 elements were for [x, y, z, wx, wy, wz, w]
            self.foot_J[i] = foot_jacobian[:,i*3+7:i*3+10]

            # foot velocity B_v_i
            self.foot_vel[:,i] = foot_jacobian @ joint_velocity
            # print(f"{i} foot position {self.foot_pos[:,i]}")
            # print(f"{i} foot velocity {self.foot_vel[:,i]}")
            # print(f"{i} foot jacobian {self.foot_J[i]}")

    # update the sensory readings, all 4 foot positions, jacobians, bias
    def update(self, jointstate_msg: JointState, odom_msg: Odometry):
        self.update_pose(odom_msg)
        self.update_joints(jointstate_msg)
        self.update_feet()
