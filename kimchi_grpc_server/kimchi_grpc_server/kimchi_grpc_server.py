import kimchi_grpc_server.kimchi_pb2 as kimchi_pb2
import kimchi_grpc_server.kimchi_pb2_grpc as kimchi_pb2_grpc
from kimchi_grpc_server.pose_2d import Pose2D

import grpc
import asyncio

# Define a class that will be used to serve the GetPose request
# The class will have a method that will be called by the server
# to serve the GetPose request
class KimchiGrpcServer(kimchi_pb2_grpc.KimchiAppServicer):
    def __init__(self, ros_node):
        self._ros_node = ros_node
        self._logger = ros_node.logger
        self._current_linear_vel = 0
        self._current_angular_vel = 0

    async def GetPose(
        self, request: kimchi_pb2.Empty, context: grpc.aio.ServicerContext
    ) -> kimchi_pb2.Pose:
        self._logger.info(f"Serving GetPose request {request}")
        pose = Pose2D(0, 0, 0)

        while True:
            await asyncio.sleep(0.5)
            pose = self._ros_node.protected_pose.pose
            self._logger.info(f"Sending pose {pose.x}, {pose.y}, {pose.theta}")

            yield kimchi_pb2.Pose(x = pose.x, y = pose.y, theta = pose.theta)

    def GetMap(self, request: kimchi_pb2.Empty, context: grpc.aio.ServicerContext):
        self._logger.info(f"Serving GetMap request {request}")
        return self._ros_node.get_map()

    def Move(self, request_iterator, context):
        """
        Receives a stream of Velocity messages from the client.
        
        Args:
            request_iterator: An iterator that yields Velocity Ratio objects. Velocity ratios are values from -1 to 1
            context: The RPC context
            
        Returns:
            An Empty response when the stream is complete
        """
        try:
            # Process each velocity message as it comes in
            for velocity_ratio in request_iterator:
                # Log the received velocity for debugging
                self._logger.info(f"Received velocity: linear={velocity_ratio.linear}, angular={velocity_ratio.angular}")
                self._current_linear_vel = velocity_ratio.linear
                self._current_angular_vel = velocity_ratio.angular

                 # If thee velocity is close to 0, then it was probably meant to be 0.
                if abs(velocity_ratio.linear) < 0.1:
                    self._current_linear_vel = 0.0
                if abs(velocity_ratio.angular) < 0.1:
                    self._current_angular_vel = 0.0

                self._logger.info(f"Publishing velocity: linear={self._current_linear_vel}, angular={self._current_angular_vel}")

                self._ros_node.publish_velocity(self._current_linear_vel, self._current_angular_vel)
                
            # Return empty response when the stream completes
            return kimchi_pb2.Empty()
            
        except Exception as e:
            self._logger.error(f"Error in Move RPC: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"Internal error: {str(e)}")
            return kimchi_pb2.Empty()

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
