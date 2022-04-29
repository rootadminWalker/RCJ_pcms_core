"""
MIT License

Copyright (c) 2020 rootadminWalker

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""
from typing import Any, Union

import genpy
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from . import Sensor


class Camera(Sensor):
    """
    This class provides ROS image stream integration and helper methods
    """

    def __init__(
            self,
            camera_topic: str,
            dataclass: genpy.Message,
            callback: Any,
            queue_size: int = None
    ):
        super(Camera, self).__init__()
        self.camera_topic = camera_topic
        self.dataclass = dataclass
        self.callback = callback
        self.queue_size = queue_size

        self.sub = rospy.Subscriber(
            camera_topic,
            dataclass,
            callback,
            queue_size=queue_size
        )

        self.bridge = CvBridge()
        self.dataclass_conversion_map = {
            Image: self.bridge.imgmsg_to_cv2,
            CompressedImage: self.bridge.compressed_imgmsg_to_cv2
        }

    def msg_to_image(self, msg: Union[Image, CompressedImage]):
        """
        Convert your subscribed msg into cv2 image
        Args:
            msg: Your subscribed image msg

        Returns: Your converted image in numpy
        Raises: AssertionError, if msgs parameter is either a sensor_msgs.msg.Image or sensor_msgs.msg.CompressedImage
        """
        # Assert if msg is neither a sensor_msgs.msg.Image or sensor_msgs.msg.CompressedImage
        assert msg.__class__ in self.dataclass_conversion_map, \
            "Your msg must either be sensor_msgs.msg.Image or sensor_msgs.msg.CompressedImage"

        return self.dataclass_conversion_map[msg.__class__](msg)
