from kimchi_grpc_server.pose_2d import Pose2D

class MapInfo:
    def __init__(self, width: int, height: int, resolution: float, origin: Pose2D, image: bytes):
        self._width = width
        self._height = height
        self._resolution = resolution
        self._origin = origin
        self._image = image

    @property
    def width(self):
        return self._width
    
    @property
    def height(self):
        return self._height
    
    @property
    def resolution(self):
        return self._resolution
    
    @property
    def origin(self):
        return self._origin
    
    @property
    def image(self):
        return self._image
