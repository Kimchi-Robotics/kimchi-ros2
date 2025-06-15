from threading import Thread
from time import sleep
import base64
import os

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from kimchi_interfaces.srv import MapInfo as MapInfoSrv
from kimchi_interfaces.srv import AddGoalToMission as AddGoalToMissionSrv
from kimchi_interfaces.msg import RobotState as RobotStateMsg

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


from kimchi_grpc_server.pose_2d import ProtectedPose2D, Pose2D
from kimchi_grpc_server.map_info import MapInfo
from kimchi_grpc_server.robot_state import RobotState
from kimchi_grpc_server.kimchi_grpc_server import KimchiGrpcServer
import kimchi_grpc_server.kimchi_pb2 as kimchi_pb2
import rclpy

# Node that serves as a bridge between ROS and the gRPC server.
class GrpcBridgeNode(Node):
    def __init__(self):
        super().__init__('grpc_bridge_node')
        self._protected_pose = ProtectedPose2D(Pose2D(0, 0, 0))
        self._robot_state = RobotState.NO_MAP

        # TODO: Use ROS parameters to set frame values
        self._robot_frame = 'base_link'
        self._map_frame = 'map'
        self._vel_topic = '/cmd_vel'
        self._max_linear_vel_ms = 0.5
        self._man_angular_vel_rad = 1

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self._robot_state_subscription = self.create_subscription(
            RobotStateMsg,
            '/kimchi_state_server/state',
            self.robot_state_callback,
            qos_profile)

        # Create velocity publisher
        self._vel_publisher = self.create_publisher(Twist, self._vel_topic, 10)
        self._map_info_client = self.create_client(
            MapInfoSrv, '/kimchi_map/get_map_info')
        self._start_mapping_client = self.create_client(
            Trigger, '/kimchi_state_server/start_slam')
        self._start_navigation_client = self.create_client(
            Trigger, '/kimchi_state_server/start_navigation')
        self._add_goal_to_mission_client = self.create_client(
            AddGoalToMissionSrv, '/kimchi_state_server/add_goal_to_mission')

    @property
    def logger(self):
        return self.get_logger()

    @property
    def protected_pose(self):
        return self._protected_pose

    def subscribe_to_robot_pose(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self._robot_frame
        to_frame_rel = self._map_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        while True:
            try:
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time(),
                    rclpy.time.Duration(seconds=1.0))
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                continue

            self._protected_pose.set(Pose2D(
                t.transform.translation.x,
                t.transform.translation.y,
                0
            ))

            sleep(0.5)

    def robot_state_callback(self, msg):
        self.get_logger().info(f'robot_state_callback called')
        self.get_logger().info(
            f'Got robot state {msg.state} converted to {RobotState.from_kimchi_robot_state_enum(msg.state)}')

        self._robot_state = RobotState.from_kimchi_robot_state_enum(msg.state)

    def publish_velocity(self, linear_percentage, angular_percentage):
        msg = Twist()
        msg.linear.x = self._max_linear_vel_ms * linear_percentage
        msg.angular.z = self._man_angular_vel_rad * angular_percentage
        self.get_logger().info(
            f'Publishing velocity. linear: {msg.linear.x}, angular {msg.angular.z}')
        self._vel_publisher.publish(msg)

    def start_mapping(self):
        self._start_mapping_client.wait_for_service()
        request = Trigger.Request()
        self.get_logger().info('Calling start mapping service')
        self._start_mapping_client.call(request)
        return [True, 'Mapping started successfully']

    def start_navigation(self):
        self._start_navigation_client.wait_for_service()
        request = Trigger.Request()
        self.get_logger().info('Calling start navigation service')
        self._start_navigation_client.call(request)
        self.get_logger().info('Finished calling start navigation service')
        return [True, 'Navigation started successfully']

    def get_map(self):
        self._map_info_client.wait_for_service()
        self.get_logger().info('Calling map info service')
        request = MapInfoSrv.Request()
        request.str_place_holder = "May you share the mapo info please?"

        response = self._map_info_client.call(request)

        if response.success is True:
            self.get_logger().info('Map info obtained')
        else:
            self.get_logger().error('Map info service returned empty map')

        map_info = kimchi_pb2.Map(
            image=bytes(response.map_image),
            resolution=response.resolution,
            origin=kimchi_pb2.Pose(
                x=-response.origin.x, y=-response.origin.y, theta=response.origin.theta)
        )

        self.get_logger().info('Sending Map')

        return map_info

    def get_robot_state(self):
        return self._robot_state

    def process_selected_pose(self, pose: Pose2D):
        """
        Processes the selected pose by updating the protected pose.

        Args:
            pose: A Pose2D object representing the selected pose.
        """
        self.get_logger().info(f'Processing selected pose: {pose.x}, {pose.y}, {pose.theta}')

        if self._robot_state == RobotState.LOCATING:
            self.get_logger().info(
                'Robot is locating. This pose will be used to set an aprox initial pose to the robot.')
        elif self._robot_state == RobotState.IDLE or self._robot_state == RobotState.NAVIGATING:
            self.get_logger().info(
                'Robot state is IDLE. The robot will be send to this pose.')

            self._add_goal_to_mission_client.wait_for_service()
            request = AddGoalToMissionSrv.Request()
            request.goal.x = pose.x
            request.goal.y = pose.y

            self._add_goal_to_mission_client.call(request)

        else:
            self.get_logger().info(
                'Robot is not doing anything.')


def main():
    rclpy.init()
    node = GrpcBridgeNode()
    kimchi_app = KimchiGrpcServer(node)

    t1 = Thread(target=node.subscribe_to_robot_pose)
    t1.daemon = True
    t1.start()

    t2 = Thread(target=kimchi_app.async_serve)
    t2.daemon = True
    t2.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
