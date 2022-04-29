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

import rospy
from geometry_msgs.msg import PoseStamped

from . import Hardware


class SLAMController(Hardware):
    def __init__(self, config=None):
        super()._check_status()
        if config is not None:
            self.config = self.__load_config(config)

        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal',
            PoseStamped,
            queue_size=1
        )

    def go_to_point(self, x, y, z, w, wait_until_end=False):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = z
        msg.pose.orientation.w = w

        while rospy.get_param('/status_monitor/status_code') != 0:
            self.goal_pub.publish(msg)

        if wait_until_end:
            while rospy.get_param('/status_monitor/status_code') != 3:
                continue

    def go_to_loc(self, loc, wait_until_end=False):
        x, y, z, w = self.config[loc]
        self.go_to_point(x, y, z, w, wait_until_end)

    @staticmethod
    def __load_config(config_path):
        return json.load(open(config_path, 'r'))
