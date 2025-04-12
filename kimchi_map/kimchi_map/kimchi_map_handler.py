import os
import time
import base64
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import SaveMap
from geometry_msgs.msg import Pose2D

from kimchi_map.map_info import MapInfo
from kimchi_interfaces.srv import MapInfo as MapInfoSrv

package_name = 'kimchi_map'

# Node that serves as a bridge between ROS and the gRPC server.
class KimchiMapHandler(Node):
    def __init__(self):
        super().__init__('kimchi_map_handler')

        # TODO: Use ROS parameters to set frame values
        self._robot_frame = 'base_link'
        self._map_frame = 'map'
        self._map_file_name = 'kimchi_map'
        self._map_file_format = 'png'
        self._map_topic = '/map'

        # Subscribe to the map topic to get the map info
        self._map_subscription = None
        self._map_info = None
        self.init_map_info()
        # self.on_map_changed()

        self._map_saver_client = self.create_client(SaveMap, '/map_saver/save_map')

        self.srv = self.create_service(MapInfoSrv, '/kimchi_map/get_map_info', self.get_map_info_callback)

    # def map_info_callback(self, msg):
    #     self.get_logger().info(f'map_info_callback called')
    #     self.get_logger().info(f'Got map with width {msg.info.width}, height {msg.info.height}, resolution {msg.info.resolution}, origin {msg.info.origin}')

    #     # Get map info.
    #     self._map_info = MapInfo(
    #         width = msg.info.width,
    #         height = msg.info.height,
    #         resolution = msg.info.resolution,
    #         origin = Pose2D(x = msg.info.origin.position.x, y = msg.info.origin.position.y, theta = 0.0),
    #         image = None
    #     )

    #     # Unsubscribe from the map topic to avoid receiving the same map info.
    #     self.destroy_subscription(self._map_subscription)
    #     self._map_subscription = None



    # def on_map_changed(self):
    #     self._map_info = None

    #     if self._map_subscription is None:
    #         self._map_subscription = self.create_subscription(
    #                     OccupancyGrid,
    #                     '/map',
    #                     self.map_info_callback,
    #                     10)

    def init_map_info(self):
        map_image_bytes = self.get_map_image_bytes()
        if map_image_bytes is None:
            self.get_logger().error('Failed to get map image bytes')
            return

        map_data_filename = os.path.abspath(f"{self._map_file_name}.yaml")
        try:
            with open(map_data_filename, "r") as map_data_file:
                values_yaml = yaml.load(map_data_file, Loader=yaml.FullLoader)
                self._map_info = MapInfo(
                    resolution = values_yaml['resolution'],
                    origin = Pose2D(x = float(values_yaml['origin'][0]), y = float(values_yaml['origin'][1]), theta = float(values_yaml['origin'][2])),
                    image = map_image_bytes
                )
        except Exception as e:
            self.get_logger().error(f'Failed load map data from {filename}: {e}')
            return
        self.get_logger().info('Map info initialized')

    def get_map_info_callback(self, request, response):
        self.get_logger().info('Executing goal...')

        # Maybe wait 5 secs?
        if self._map_info is None:
            self.get_logger().error('Map not yet initialized')
            response.success = False
            return response

        self.get_logger().info('Got map image')

        response.map_image = self._map_info.image
        response.resolution = self._map_info.resolution
        response.origin = Pose2D()
        response.origin.x = self._map_info.origin.x
        response.origin.y = self._map_info.origin.y
        response.origin.theta = self._map_info.origin.theta
        response.success = True

        return response

    def get_map_image_bytes(self):
        # Save Map image as array of bytes (base 64).
        filename = os.path.abspath(f"{self._map_file_name}.{self._map_file_format}")
        try:
            with open(filename, "rb") as image_file:
                self.get_logger().info('Encoding image')
                map_bytes = base64.b64encode(image_file.read())
        except Exception as e:
            self.get_logger().error(f'Failed to encode image from file {filename}: {e}')
            return None

        return map_bytes

    # TODO: Make this a service
    def save_map(self):
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

        self.get_logger().info(f'Saving map to {self._map_file_name}.{self._map_file_format}')
 
        self.future = self._map_saver_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
 
        self.get_logger().info('Finished saving map')
        if response.result is True:
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
