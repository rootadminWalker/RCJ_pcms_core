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
import json

import dlib
import genpy
import numpy as np
from home_robot_msgs.msg import ObjectBox

from . import SensorFuncWrapper
from ..Dtypes.FaceProcess import FaceUser
from ..utils.FaceUserManager import FaceUserManager


class FaceRecognition(SensorFuncWrapper):
    """
    Face descriptor parsing object.
    """
    def __init__(
            self,
            landmark_vector_convertor: dlib.face_recognition_model_v1,
            landmark_predictor: dlib.shape_predictor
    ):
        """
        Args:
            landmark_vector_convertor: The landmark_vector_convertor instance
            landmark_predictor: The landmark_predictor instance
        """
        self.landmark_vector_convertor = landmark_vector_convertor
        self.landmark_predictor = landmark_predictor

    def recognize_face(self, image: np.array, box: dlib.rectangle, user_manager: FaceUserManager) -> FaceUser:
        descriptor = self.parse_face_descriptor(image, box)
        temp_user = FaceUser('', descriptor=descriptor)
        recognized_user = user_manager.sign_in(temp_user)

        return recognized_user

    def parse_face_descriptor(self, image: np.array, box: dlib.rectangle) -> np.array:
        """
        This method can parse the face descriptor from an image
        Args:
            image: The FULL image that contains the face you want to parse
            box: A dlib.rectangle whose coordinates are the face location in your FULL image

        Returns:
            The descriptor parsed with the two models
        """
        landmarks = self.landmark_predictor(image, box)
        descriptor = self.landmark_vector_convertor.compute_face_descriptor(image, landmarks)

        return descriptor

    def serialize(self, box: dlib.rectangle, recognized_user: FaceUser) -> genpy.Message:
        """
        This method serializes the output of self.recognize_face into an ObjectBox message
        Args:
            box: The face box coordinates
            recognized_user: FaceUser dtype that contains the username and descriptor

        Returns:

        """
        serialized_box = ObjectBox()
        serialized_box.model = 'dlib_face_recognition'

        serialized_box.label = recognized_user.username
        # Pump the descriptor into custom_data
        serialized_box.custom_data = json.dumps(recognized_user.descriptor.tolist())

        serialized_box.x1 = box.left()
        serialized_box.y1 = box.top()
        serialized_box.x2 = box.right()
        serialized_box.y2 = box.bottom()

        return serialized_box
