from threading import Thread
from time import sleep
import base64

from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid
from kimchi_grpc_server.pose_2d import ProtectedPose2D, Pose2D
from kimchi_grpc_server.map_info import MapInfo
from kimchi_grpc_server.kimchi_grpc_server import KimchiGrpcServer
import kimchi_grpc_server.kimchi_pb2 as kimchi_pb2
import rclpy

# Node that serves as a bridge between ROS and the gRPC server.
class GrpcBridgeNode(Node):
    def __init__(self):
        super().__init__('grpc_bridge_node')
        self._protected_pose = ProtectedPose2D(Pose2D(0, 0, 0))

        # Subscribe to the map topic to get the map info
        self._map_subscription = None
        self._map_info = None
        self.on_map_changed()

        # TODO: Use ROS parameters to set frame values
        self._robot_frame = 'base_link'
        self._map_frame = 'map'
        self._map_file_name = 'kimchi_map'
        self._map_file_format = 'png'
        self._map_topic = '/map'

        self._map_saver_client = self.create_client(SaveMap, '/map_saver/save_map')

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

    def map_info_callback(self, msg):
        self.get_logger().info(f'map_info_callback called')
        self.get_logger().info(f'Got map with width {msg.info.width}, height {msg.info.height}, resolution {msg.info.resolution}, origin {msg.info.origin}')

        # Get map info.
        self._map_info = MapInfo(
            width = msg.info.width,
            height = msg.info.height,
            resolution = msg.info.resolution,
            origin = Pose2D(x = msg.info.origin.position.x, y = msg.info.origin.position.y, theta = 0),
            image = None
        )

        # Unsubscribe from the map topic to avoid receiving the same map info.
        self.destroy_subscription(self._map_subscription)
        self._map_subscription = None

    def on_map_changed(self):
        self._map_info = None

        if self._map_subscription is None:
            self._map_subscription = self.create_subscription(
                        OccupancyGrid,
                        '/map',
                        self.map_info_callback,
                        10)

    def get_map(self):
        if self._map_info is None:
            self.get_logger().error('Map not yet initialized')
            return None

        # Save Map image as file.
        self._map_saver_client.wait_for_service()
        self.get_logger().info('Calling save map service')
        request = SaveMap.Request()
        request.map_topic = self._map_topic
        request.map_url = self._map_file_name
        request.image_format = self._map_file_format
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        response = self._map_saver_client.call(request)
        response.result

        self.get_logger().info('Finished saving map')
        if response.result is True:
            self.get_logger().info('Map saved')
        else:
            self.get_logger().error('Failed to save map')

        # Save Map image as array of bytes (base 64).
        filename = f'/home/arilow/ws/{self._map_file_name}.{self._map_file_format}'
        try:
            with open(filename, "rb") as image_file:
                self.get_logger().error('Encoding image')
                map_bytes = base64.b64encode(image_file.read())
        except Exception as e:
            self.get_logger().error(f'Failed to encode image from file {filename}: {e}')
            return None

        # Multiply the width and height by the resolution to get the size in meters.
        map_info = kimchi_pb2.Map(
            image = map_bytes,
            resolution = self._map_info.resolution,
            origin = kimchi_pb2.Pose(x = -self._map_info.origin.x, y = -self._map_info.origin.y, theta = self._map_info.origin.theta)
        )

        return map_info

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
