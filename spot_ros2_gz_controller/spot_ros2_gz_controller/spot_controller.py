import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger

from .robot_state import RobotState

class SpotController(Node):
    def __init__(self):
        super().__init__('spot_controller')
 
        # Store current robot states
        self.robot_state = RobotState()

        # Store current joint states
        self.current_positions = {}
        self.current_velocities = {}  
    
        # Create subscriber for joint states
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # Create subscriber for odometry
        self.joint_states_sub = self.create_subscription(
            Odometry,
            '/spot/odometry',
            self.odometry_callback,
            10
        )

        # Create publisher for joint trajectory
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/spot/joint_trajectory',
            10
        )
       
        # Create services
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

        # For debugging        
        self.create_timer(1.0, self.control_loop)

        self.get_logger().info('Spot controller initialized.')


    def joint_states_callback(self, msg: JointState):
        self.robot_state.update_joints(msg)
    
    def odometry_callback(self, msg: Odometry):
        self.robot_state.update_pose(msg)

    def control_loop(self):
        print(self.robot_state)

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