from abc import ABC, abstractmethod
from typing import Any

import genpy


class SensorFuncWrapper(ABC):
    """
    Sensor function wrapper for methods like detection
    """
    @abstractmethod
    def serialize(self, *args, **kwargs) -> genpy.Message:
        """
        Serialize the output into ROS format
        Returns:
            A genpy.Message
        """
        pass


from .YOLOv5 import YOLOV5
from .PersonReidentification import PersonReidentification
