import kimchi_grpc_server.kimchi_pb2 as kimchi_pb2
import kimchi_grpc_server.kimchi_pb2_grpc as kimchi_pb2_grpc
from kimchi_grpc_server.pose_2d import Pose2D

import grpc
import asyncio

# Format with ctrl + shift + i

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

            yield kimchi_pb2.Pose(x=pose.x, y=pose.y, theta=pose.theta)

    async def SubscribeToMap(
        self, request: kimchi_pb2.Empty, context: grpc.aio.ServicerContext
    ) -> kimchi_pb2.Map:
        self._logger.info(f"Serving SubscribeToMap request {request}")

        # Here!!!
        # TODO: Subscribe to /map topic and use conditional to retrieve a map when it changes
        # https://colab.research.google.com/drive/1pcEI-5QQyqWRe69PIYY2SM1ISZX3pnW8#scrollTo=O_UzvRLUYRwV
        while True:
            await asyncio.sleep(0.5)
            map_info = self._ros_node.get_map()
            self._logger.info(f"Sending map {map_info}")

            yield kimchi_pb2.Map(image=map_info.image, origin=map_info.origin, resolution=map_info.resolution)

    async def SubscribeToRobotState(
        self, request: kimchi_pb2.Empty, context: grpc.aio.ServicerContext
    ) -> kimchi_pb2.RobotStateMsg:
        self._logger.info(f"Serving SubscribeToRobotState request {request}")
        old_robot_state = None
        while True:
            await asyncio.sleep(0.5)
            robot_state = self._ros_node.get_robot_state()
            if robot_state != old_robot_state:
                old_robot_state = robot_state
                yield kimchi_pb2.RobotStateMsg(state=robot_state.to_kimchi_robot_state_enum())
    
    def GetMap(self, request: kimchi_pb2.Empty, context: grpc.aio.ServicerContext):
        self._logger.info(f"Serving GetMap request {request}")
        return self._ros_node.get_map()

    def GetRobotState(self, request: kimchi_pb2.Empty, context: grpc.aio.ServicerContext):
        self._logger.info(f"Serving GetRobotState request {request}")
        robot_state = self._ros_node.get_robot_state()
        self._logger.info(f"Sending robot state {robot_state}")
        return kimchi_pb2.RobotStateMsg(state=robot_state.to_kimchi_robot_state_enum())

    def StartMapping(self, request: kimchi_pb2.Empty, context: grpc.aio.ServicerContext):
        self._logger.info(f"Serving StartMapping request {request}!!!!!!!!!!!!!!!!!!!!!!!")
        [success, msg]= self._ros_node.start_mapping()
        self._logger.info(f"Finished calling start mapping service!!!!!!!!!!!!!!!!!!!!!!!!1")
        return kimchi_pb2.StartMappingResponse(success=success, info=msg)

    def StartNavigation(self, request: kimchi_pb2.Empty, context: grpc.aio.ServicerContext):
        self._logger.info(f"Serving StartNavigation request {request}")
        [success, msg] = self._ros_node.start_navigation()
        return kimchi_pb2.StartNavigationResponse(success=success, info=msg)

    def IsAlive(self, request: kimchi_pb2.Empty, context: grpc.aio.ServicerContext):
        self._logger.info(f"Serving IsAlive request {request}")
        return kimchi_pb2.IsAliveResponse(alive=True)

    def SendSelectedPose(self, request: kimchi_pb2.Pose, context: grpc.aio.ServicerContext):
        """
        Receives a Pose message from the client and sends it to the ROS node.

        Args:
            request: A Pose message containing the selected pose
            context: The RPC context

        Returns:
            An Empty response
        """
        try:
            self._logger.info(f"Received selected pose: {request.x}, {request.y}, {request.theta}")
            self._ros_node.process_selected_pose(Pose2D(request.x, request.y, request.theta))
            return kimchi_pb2.Empty()
        except Exception as e:
            self._logger.error(f"Error in SendSelectedPose RPC: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"Internal error: {str(e)}")
            return kimchi_pb2.Empty()
    
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
                self._logger.info(
                    f"Received velocity: linear={velocity_ratio.linear}, angular={velocity_ratio.angular}")
                self._current_linear_vel = velocity_ratio.linear
                self._current_angular_vel = velocity_ratio.angular

                # If thee velocity is close to 0, then it was probably meant to be 0.
                if abs(velocity_ratio.linear) < 0.1:
                    self._current_linear_vel = 0.0
                if abs(velocity_ratio.angular) < 0.1:
                    self._current_angular_vel = 0.0

                self._logger.info(
                    f"Publishing velocity: linear={self._current_linear_vel}, angular={self._current_angular_vel}")

                self._ros_node.publish_velocity(
                    self._current_linear_vel, self._current_angular_vel)

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
