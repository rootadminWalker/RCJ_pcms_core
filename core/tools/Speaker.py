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
import warnings

import rospy
from mr_voice.srv import SpeakerSrv
from std_msgs.msg import String

from .Abstract import Tools
from .FacialDisplayController import FacialDisplayController


class Speaker(Tools):
    SPEAKER_SRV_TIMEOUT = 6

    def __init__(self, speaker_topic='/speaker/say', speaker_srv='/speaker/text'):
        super()._check_status()

        self.facial_controller = FacialDisplayController()
        try:
            rospy.wait_for_service(speaker_srv, timeout=Speaker.SPEAKER_SRV_TIMEOUT)
            self.speaker_srv = rospy.ServiceProxy(speaker_srv, SpeakerSrv)
        except rospy.exceptions.ROSException:
            warnings.warn(
                "Currently service speaker wasn't available, calling say_until_end will do nothing")
            self.speaker_srv = lambda x: x

        self.speaker_pub = rospy.Publisher(
            speaker_topic,
            String,
            queue_size=1
        )

    def say(self, text):
        self.speaker_pub.publish(text)
        self.facial_controller.change_emotion(text, 'happy-2')

    def say_until_end(self, text):
        self.speaker_srv(text)
        self.facial_controller.change_emotion(text, 'happy-2')
