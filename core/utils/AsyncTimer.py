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

from ..base_classes import Unit
import rospy


class AsyncTimer(Unit):
    def __init__(self, duration, auto_start=False):
        super()._check_status()
        self.duration = rospy.Duration(duration)
        self.timer = None

        self.timer_started = False
        self.timer_ended = False

        if auto_start:
            self.start()

    def is_timer_started(self):
        return self.timer_started or self.timer_ended

    def is_timer_ended(self):
        if not self.is_timer_started():
            raise RuntimeError("Timer not started yet")

        return self.timer_ended

    def __timer_callback(self, _):
        self.timer_ended = True
        self.timer_started = False

    def start(self):
        if self.is_timer_started():
            raise RuntimeError("Please reset the timer first")

        self.timer_started = True
        self.timer_ended = False
        self.timer = rospy.Timer(
            self.duration,
            self.__timer_callback,
            oneshot=True
        )

    def reset(self):
        if not self.is_timer_started():
            raise RuntimeError("Timer not started yet")

        self.timer.shutdown()
        self.timer_started = False
        self.timer_ended = False
