## kimchi_grpc_server

gRPC server that handles requests from the Kimchi Mobile App.

### Services

- GetPose: Stream pose of the robot.
- Move: Receive stream of velocity commands as ratio (values between -1 and 1)to be sent to the robot.
- GetMap: Get the map of the robot.

### Build proto files

prerequisites:
- grpcio-tools
- protobuf

Running `colcon build` already builds the proto files inside ./kimchi_grpc_server/proto directory. If you want to modify the proto and use it:
- Copy the generated files from ./kimchi_grpc_server/proto to ./kimchi_grpc_server.
- In kimchi_pb2_grpc.py, modify  `import kimchi_pb2 as kimchi__pb2` to be `import kimchi_grpc_server.proto.kimchi_pb2 as kimchi__pb2`
- Also remember to update the proto files on the client side (Probably the mobile app)

Unfortunately, the proto build doesn't let you do this, so it has to be done manually.
