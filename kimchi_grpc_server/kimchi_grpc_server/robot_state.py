from enum import Enum

import kimchi_grpc_server.kimchi_pb2 as kimchi_pb2


class RobotState(Enum):
    NO_MAP = 0
    MAPPING_WITH_EXPLORATION = 1
    MAPPING_WITH_TELEOP = 2
    NAVIGATING = 3
    TELEOP = 4
    IDLE = 5

    def to_kimchi_robot_state_enum(self):
        kimchi_robot_state = kimchi_pb2.RobotStateEnum.NO_MAP
        if self == RobotState.NO_MAP:
            kimchi_robot_state = kimchi_pb2.RobotStateEnum.NO_MAP
        elif self == RobotState.MAPPING_WITH_EXPLORATION:
            kimchi_robot_state = kimchi_pb2.RobotStateEnum.MAPPING_WITH_EXPLORATION
        elif self == RobotState.MAPPING_WITH_TELEOP:
            kimchi_robot_state = kimchi_pb2.RobotStateEnum.MAPPING_WITH_TELEOP
        elif self == RobotState.NAVIGATING:
            kimchi_robot_state = kimchi_pb2.RobotStateEnum.NAVIGATING
        elif self == RobotState.TELEOP:
            kimchi_robot_state = kimchi_pb2.RobotStateEnum.TELEOP
        elif self == RobotState.IDLE:
            kimchi_robot_state = kimchi_pb2.RobotStateEnum.IDLE
        return kimchi_robot_state

    def from_kimchi_robot_state_enum(robot_state_enum):
        robot_state = RobotState.NO_MAP
        if robot_state_enum is kimchi_pb2.RobotStateEnum.NO_MAP:
            robot_state = RobotState.NO_MAP
        elif robot_state_enum is kimchi_pb2.RobotStateEnum.MAPPING_WITH_EXPLORATION:
            robot_state = RobotState.MAPPING_WITH_EXPLORATION
        elif robot_state_enum is kimchi_pb2.RobotStateEnum.MAPPING_WITH_TELEOP:
            robot_state = RobotState.MAPPING_WITH_TELEOP
        elif robot_state_enum is kimchi_pb2.RobotStateEnum.NAVIGATING:
            robot_state = RobotState.NAVIGATING
        elif robot_state_enum is kimchi_pb2.RobotStateEnum.TELEOP:
            robot_state = RobotState.TELEOP
        elif robot_state_enum is kimchi_pb2.RobotStateEnum.IDLE:
            robot_state = RobotState.IDLE
        return robot_state
