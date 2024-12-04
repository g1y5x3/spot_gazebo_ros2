import numpy as np
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class RobotState:
    def __init__(self):

        # fixed order of the joints
        self.joint_names = [
            'front_left_hip_x', 'front_left_hip_y', 'front_left_knee',
            'front_right_hip_x', 'front_right_hip_y', 'front_right_knee',
            'rear_left_hip_x', 'rear_left_hip_y', 'rear_left_knee',
            'rear_right_hip_x', 'rear_right_hip_y', 'rear_right_knee'
        ]

        # joint states
        self.q = np.zeros(12)
        self.q_dot = np.zeros(12)
        
        # position and orientation states
        self.theta = np.zeros(3)
        self.p = np.zeros(3)
        self.omega = np.zeros(3)    # angular velocity
        self.p_dot = np.zeros(3)    # linear velocity
        
    def update_joints(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.q[idx] = msg.position[i]
                self.q_dot[idx] = msg.velocity[i]

    # TODO: estimate the state based on sensors inputs to replace ground truth
    # provided by gazebo 
    def update_pose(self, msg: Odometry):
        # position and pose velocity
        p = msg.pose.pose.position
        v = msg.twist.twist.linear

        self.p = np.array([p.x, p.y, p.z])
        self.p_dot = np.array([v.x, v.y, v.z])

        # orientation and angular velocity
        q = msg.pose.pose.orientation
        ang_vel = msg.twist.twist.angular

        # convert quaternion to euler angles
        r = R.from_quat([q.x, q.y, q.z, q.w])
        self.theta = r.as_euler('zyx', degrees=False)
        self.omega = np.array([ang_vel.z, ang_vel.y, ang_vel.x])

    def __str__(self):
        output = "Robot State:\n"
        output += "joint positions (q):\n"
        for i, name in enumerate(self.joint_names):
            output += f"{name}: {self.q[i]:.3f}\n"

        output += "\njoint velocities (q_dot):\n"  
        for i, name in enumerate(self.joint_names):
            output += f"{name}: {self.q_dot[i]:.3f}\n"

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

