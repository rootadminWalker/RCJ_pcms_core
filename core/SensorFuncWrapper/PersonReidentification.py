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
import cv2 as cv
import numpy as np
from . import SensorFuncWrapper


class PersonReidentification(SensorFuncWrapper):
    def __init__(self, bin_path, xml_path):
        self.person_extractor = cv.dnn.readNet(bin_path, xml_path)

    def extract_descriptor(self, person_image, crop=False) -> np.array:
        blob = cv.dnn.blobFromImage(
            person_image,
            size=(128, 256),
            scalefactor=1.0,
            mean=(0, 0, 0),
            swapRB=False,
            crop=crop
        )

        self.person_extractor.setInput(blob)
        descriptor = self.person_extractor.forward()
        return descriptor

    @staticmethod
    def compare_descriptors(desc1, desc2):
        return np.dot(desc1, desc2) / (np.linalg.norm(desc1) * np.linalg.norm(desc2))

    def serialize(self, *args, **kwargs):
        pass
