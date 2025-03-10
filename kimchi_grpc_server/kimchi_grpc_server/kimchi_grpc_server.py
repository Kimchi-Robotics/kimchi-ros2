import kimchi_grpc_server.kimchi_pb2 as kimchi_pb2
import kimchi_grpc_server.kimchi_pb2_grpc as kimchi_pb2_grpc
from kimchi_grpc_server.pose_2d import Pose2D

import grpc
import asyncio

from time import sleep

# Define a class that will be used to serve the GetPose request
# The class will have a method that will be called by the server
# to serve the GetPose request
class KimchiGrpcServer(kimchi_pb2_grpc.KimchiAppServicer):
    def __init__(self, ros_node):
        self._ros_node = ros_node
        self._logger = ros_node.logger

    async def GetPose(
        self, request: kimchi_pb2.Empty, context: grpc.aio.ServicerContext
    ) -> kimchi_pb2.Pose:
        self._logger.info(f"Serving GetPose request {request}")
        pose = Pose2D(0, 0, 0)

        while True:
            pose = self._ros_node.protected_pose.pose
            self._logger.info(f"Sending pose {pose.x}, {pose.y}, {pose.theta}")
            sleep(0.5)

            yield kimchi_pb2.Pose(x = pose.x, y = pose.y, theta = pose.theta)

    def GetMap(self, request: kimchi_pb2.Empty, context: grpc.aio.ServicerContext):
        self._logger.info(f"Serving GetMap request {request}")
        return self._ros_node.get_map()
        
        # return super().GetMap(request, context)

    def async_serve(self):
        asyncio.run(self.serve())

    async def serve(self) -> None:
        server = grpc.aio.server()
        kimchi_pb2_grpc.add_KimchiAppServicer_to_server(self, server)
        listen_addr = "0.0.0.0:50051"
        server.add_insecure_port(listen_addr)
        self._logger.info(f"Starting server on {listen_addr}")
        await server.start()
        await server.wait_for_termination()
