import rclpy, time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger
from typing import Optional
from pathlib import Path

from .robot_state import RobotState
from .gait_scheduler import GaitScheduler
from .swing_trajectory import SwingTrajectory
from .mpc_controller import MPCController
from .leg_controller import LegController

class SpotController(Node):
    def __init__(self):
        super().__init__('spot_controller')

        # Service TODO: separate the service into a different file
        # joint names in order
        self.joint_names = [
            'front_left_hip_x',  'front_left_hip_y',  'front_left_knee',
            'front_right_hip_x', 'front_right_hip_y', 'front_right_knee',
            'rear_left_hip_x',   'rear_left_hip_y',   'rear_left_knee',
            'rear_right_hip_x',  'rear_right_hip_y',  'rear_right_knee'
        ]

        # joint positions for standing pose
        self.standing_pose = [
            0.0, 0.5, -1.0,  # front left leg
            0.0, 0.5, -1.0,  # front right leg
            0.0, 0.5, -1.0,  # rear left leg
            0.0, 0.5, -1.0   # rear right leg
        ]

        # Joint positions for sitting pose
        self.sitting_pose = [
            0.0, 1.5, -2.7,  # front left leg
            0.0, 1.5, -2.7,  # front right leg
            0.0, 1.5, -2.7,  # rear left leg
            0.0, 1.5, -2.7   # rear right leg
        ]

        # Create callback group for services
        self.callback_group = ReentrantCallbackGroup()

        self.stand_service = self.create_service(
            Trigger,
            'spot/stand',
            self.handle_stand,
            callback_group=self.callback_group
        )

        self.sit_service = self.create_service(
            Trigger,
            'spot/sit',
            self.handle_sit,
            callback_group=self.callback_group
        )

        # Control subscribe to /cmd_vel and publish to /joint_trajectory
        spot_model_description = get_package_share_directory('spot_ros2_description')

        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.joint_states_sub = self.create_subscription(
            Odometry,
            '/spot/odometry',
            self.odometry_callback,
            10
        )

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/spot/joint_trajectory',
            10
        )

        self.clock_msg: Optional[Clock] = None
        self.last_jointstate_msg: Optional[JointState] = None
        self.last_odometry_msg: Optional[Odometry] = None

        self.robot_state = RobotState(str(Path(spot_model_description) / 'models' / 'spot' / 'model.sdf'))
        self.initialized = False

        self.create_timer(1/20, self.controller_callback)
        # self.create_timer(1/4500.0, self.leg_control_callback)

    def clock_callback(self, msg: Clock):
        self.clock_msg = msg

    def joint_states_callback(self, msg: JointState):
        self.last_jointstate_msg = msg

    def odometry_callback(self, msg: Odometry):
        self.last_odometry_msg = msg

    def controller_callback(self):
        # wait for the simulation environment to start to initialize the controller
        if (self.clock_msg is None or
            self.clock_msg.clock.nanosec == 0 or
            self.last_jointstate_msg is None or
            self.last_odometry_msg is None):
            return

        if not self.initialized:
            self.get_logger().info("Initializing controller...")
            time.sleep(3.0)

            self.robot_state.update(self.last_jointstate_msg, self.last_odometry_msg)
            # self.gait_scheduler = GaitScheduler(gait_cycle=2, horizon=16, start_time=self.get_clock().now())
            self.gait_scheduler = GaitScheduler(start_time=self.get_clock().now())
            self.mpc_controller = MPCController(self.robot_state, self.gait_scheduler)
            self.swing_trajectory_generator = SwingTrajectory(swing_height=0.2)
            self.leg_controller = LegController()

            self.initialized = True
            self.get_logger().info('Spot controller initialized.')       
            time.sleep(3.0) # just to have time to inspect all the initialization messages
            return

        self.robot_state.update(self.last_jointstate_msg, self.last_odometry_msg)
        self.gait_scheduler.update_phase(self.get_clock().now())
        # # TODO take cmd_vel for desired p_dot
        com_vel = [0.0, 0.0, 0.0]
        self.mpc_controller.update_control(com_vel, self.robot_state, self.gait_scheduler)
        # self.swing_trajectory_generator.update_swingfoot_trajectory(com_vel, self.robot_state, self.gait_scheduler)

        self.leg_controller.update(self.trajectory_pub, 
                                   self.robot_state, self.gait_scheduler, 
                                   self.swing_trajectory_generator, self.mpc_controller)

    # Service
    def publish_trajectory(self, positions, duration=2.0):
        """Publish a joint trajectory."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)

        msg.points = [point]
        self.trajectory_pub.publish(msg)

    def handle_stand(self, request, response):
        """Handle stand service request."""
        self.get_logger().info('Standing up...')
        self.publish_trajectory(self.standing_pose, duration=5.0)
        response.success = True
        response.message = "Standing up command sent"
        return response

    def handle_sit(self, request, response):
        """Handle sit service request."""
        self.get_logger().info('Sitting down...')
        self.publish_trajectory(self.sitting_pose, duration=5.0)
        response.success = True
        response.message = "Sitting down command sent"
        return response

def main(args=None):
    rclpy.init(args=args)
    controller = SpotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()