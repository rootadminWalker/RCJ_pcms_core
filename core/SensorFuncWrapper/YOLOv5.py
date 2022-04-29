#!/usr/bin/env python3
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
from typing import Union

import numpy as np
import torch
from cv_bridge import CvBridge
from home_robot_msgs.msg import ObjectBoxes, ObjectBox

from core.SensorFuncWrapper import SensorFuncWrapper


class YOLOV5(SensorFuncWrapper):
    """
    YOLOV5 detection
    """

    def __init__(self, model_type: str):
        """
        Args:
            model_type: Type of the model (yolov5n, yolov5s, yolov5m, yolov5l, yolov5x)
        """
        self.model_type = model_type
        self.model = torch.hub.load('ultralytics/yolov5', self.model_type, pretrained=True)
        self.bridge = CvBridge()

    def detect(self, image: Union[np.array, torch.tensor]):
        """
        This method parse the detection of the model and returns the results
        Args:
            image: You can input either a cv2 image, PIL Image, torch tensor even an url

        Returns: yolov5.models.common.Detection, which includes all the detection values

        """
        return self.model(image)

    def serialize(self, image: np.array, results) -> ObjectBoxes:
        """
        Serialize the results to ObjectBoxes
        Args:
            image: The original image (MUST be np.array)
            results: yolov5.models.common.Detection taken from YOLOV5().detect

        Returns: ObjectBoxes, which contains all the results

        """

        assert isinstance(image, np.ndarray), 'The image must be a np.array'
        results = results.pandas().xyxy[0]
        boxes = ObjectBoxes()
        for i in range(len(results)):
            box = ObjectBox()
            box.model = self.model_type
            box.x1 = int(results.xmin[i])
            box.y1 = int(results.ymin[i])
            box.x2 = int(results.xmax[i])
            box.y2 = int(results.ymax[i])
            box.score = results.confidence[i]
            box.label = results.name[i]

            source_img = image[box.y1:box.y2, box.x1:box.x2, :].copy()
            box.source_img = self.bridge.cv2_to_compressed_imgmsg(source_img)

            boxes.boxes.append(box)

        boxes.source_img = self.bridge.cv2_to_compressed_imgmsg(image)
        return boxes
