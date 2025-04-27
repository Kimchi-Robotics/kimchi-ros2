from threading import Lock

# Define a class that will be used to store the pose of the robot
# in the 2D plane


class Pose2D:
    def __init__(self, x: float, y: float, theta: float):
        self._x = x
        self._y = y
        self._theta = theta

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def theta(self):
        return self._theta


# Thread-safe Pose2D
class ProtectedPose2D:
    def __init__(self, pose):
        self._pose = pose
        self._mutex = Lock()

    @property
    def pose(self):
        with self._mutex:
            return self._pose

    def set(self, pose):
        with self._mutex:
            self._pose = pose
