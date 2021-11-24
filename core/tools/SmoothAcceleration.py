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

from .PIDController import PIDController


class SmoothAcceleration(PIDController):
    def __init__(self, kp, ki, kd, max_acceleration):
        super(SmoothAcceleration, self).__init__(kp, ki, kd)
        self.max_acceleration = max_acceleration

    def smooth_speed(self, current_speed, target_speed):
        target_speed_error = target_speed - current_speed
        smooth_speed = super(SmoothAcceleration, self).update(target_speed_error)
        if abs(smooth_speed) > self.max_acceleration:
            if smooth_speed < 0:
                smooth_speed = -self.max_acceleration
            else:
                smooth_speed = self.max_acceleration

        return current_speed + smooth_speed
