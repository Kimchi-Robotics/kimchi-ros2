import os
from time import sleep
import base64
import yaml
from threading import Thread, Lock
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav2_msgs.srv import SaveMap
from geometry_msgs.msg import Pose2D
from kimchi_interfaces.msg import RobotState as RobotStateMsg

from kimchi_map.map_info import MapInfo
from kimchi_interfaces.srv import MapInfo as MapInfoSrv
from kimchi_grpc_server.robot_state import RobotState

package_name = 'kimchi_map'

# Node that serves as a bridge between ROS and the gRPC server.
class KimchiMapHandler(Node):
    def __init__(self):
        super().__init__('kimchi_map_handler')
        self._robot_state = RobotState.NO_MAP

        # TODO put this in a separated class
        self._map_saver_thread = Thread(target=self.save_map_loop)
        self._map_saver_thread.daemon = True
        self._keep_saving_map = False

        # TODO: Use ROS parameters to set frame values
        self._robot_frame = 'base_link'
        self._map_frame = 'map'
        self._map_file_name = 'kimchi_map'
        self._map_file_format = 'png'
        self._map_topic = '/map'

        # Subscribe to the map topic to get the map info
        self._map_subscription = None
        self._map_info = None
        self._map_info_mutex = Lock()
        self.update_map_info(self._map_file_name)

        self._map_saver_client = self.create_client(
            SaveMap, '/map_saver/save_map')

        self._get_map_info_service = self.create_service(
            MapInfoSrv, '/kimchi_map/get_map_info', self.get_map_info_callback)

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

    def update_map_info(self, file_name):
        map_image_bytes = self.get_map_image_bytes(file_name)
        if map_image_bytes is None:
            self.get_logger().error('Failed to get map image bytes')
            return

        map_data_filename = os.path.abspath(f"{file_name}.yaml")
        try:
            with open(map_data_filename, "r") as map_data_file:
                values_yaml = yaml.load(map_data_file, Loader=yaml.FullLoader)
                with self._map_info_mutex:
                    self._map_info = MapInfo(
                        resolution=values_yaml['resolution'],
                        origin=Pose2D(x=float(values_yaml['origin'][0]), y=float(
                            values_yaml['origin'][1]), theta=float(values_yaml['origin'][2])),
                        image=map_image_bytes
                    )
        except Exception as e:
            self.get_logger().error(
                f'Failed load map data from {filename}: {e}')
            return
        self.get_logger().info('Map info initialized')

    def robot_state_callback(self, msg):
        new_state = RobotState.from_kimchi_robot_state_enum(msg.state)
        if new_state == self._robot_state:
            return

        self._robot_state = new_state
        if self._map_saver_thread.is_alive() and self._robot_state != RobotState.MAPPING_WITH_TELEOP and self._robot_state != RobotState.MAPPING_WITH_EXPLORATION:
            self.get_logger().info('Stopping map saving thread')
            self._keep_saving_map = False
            self._map_saver_thread.join()

        # Handle state change
        if new_state == RobotState.NO_MAP:
            self.get_logger().info('Robot state changed to NO_MAP')
        elif new_state == RobotState.MAPPING_WITH_TELEOP:
            self.get_logger().info('Robot state changed to MAPPING_WITH_TELEOP')
            if not self._map_saver_thread.is_alive():
                self.get_logger().info('Starting map saving thread')
                self._map_saver_thread.start()
                self._keep_saving_map = True
            else:
                self.get_logger().info('Map saving thread not started')
        elif new_state == RobotState.MAPPING_WITH_EXPLORATION:
            self.get_logger().info(
                'Robot state changed to MAPPING_WITH_EXPLORATION. Not implemented!!')
        elif new_state == RobotState.NAVIGATING:
            self.get_logger().info('Robot state changed to NAVIGATING')
        elif new_state == RobotState.TELEOP:
            self.get_logger().info('Robot state changed to TELEOP')
        elif new_state == RobotState.IDLE:
            self.get_logger().info('Robot state changed to IDLE')

    def get_map_info_callback(self, request, response):
        self.get_logger().info('Executing get_map_info_callback')

        if self._map_info is None:
            self.get_logger().error('Map not yet initialized')
            response.success = False
            return response

        with self._map_info_mutex:
            response.map_image = self._map_info.image
            response.resolution = self._map_info.resolution
            response.origin = Pose2D()
            response.origin.x = self._map_info.origin.x
            response.origin.y = self._map_info.origin.y
            response.origin.theta = self._map_info.origin.theta
            response.success = True

        return response

    def get_map_image_bytes(self, file_name):
        # Save Map image as array of bytes (base 64).
        filename = os.path.abspath(
            f"{file_name}.{self._map_file_format}")
        try:
            with open(filename, "rb") as image_file:
                self.get_logger().info('Encoding image')
                map_bytes = base64.b64encode(image_file.read())
        except Exception as e:
            self.get_logger().error(
                f'Failed to encode image from file {filename}: {e}')
            return None

        return map_bytes

    # Save map in a loop until the thread is stopped
    def save_map_loop(self):
        self.get_logger().info('Starting map saving thread')
        mapping_map_name = "current_mapping_map"
        while self._keep_saving_map:
            self.save_map_sync(mapping_map_name)
            self.update_map_info(mapping_map_name)
            sleep(1)

    def save_map_sync(self, file_name):
        # Save Map image as file.
        self._map_saver_client.wait_for_service()
        self.get_logger().info('Calling save map service')
        request = SaveMap.Request()
        request.map_topic = self._map_topic
        request.map_url = file_name
        request.image_format = self._map_file_format
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        self.get_logger().info(
            f'Saving map to {file_name}.{self._map_file_format}')

        result = self._map_saver_client.call(request)
        if result.result is True:
            self.get_logger().info('Map saved')
        else:
            self.get_logger().error('Failed to save map')


def main():
    rclpy.init()
    node = KimchiMapHandler()
    node.get_logger().info("Starting Kimchi Map node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
