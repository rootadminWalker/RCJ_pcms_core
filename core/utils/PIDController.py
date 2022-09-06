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
import time


class PIDController:
    def __init__(self, kp, ki, kd, current_time=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.last_error = 0.
        self.integ_error = 0.

        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

    def update(self, error, current_time=None):
        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time

        self.integ_error += error * delta_time
        delta_error = error - self.last_error

        d_term = 0
        if delta_time > 0:
            d_term = delta_error / delta_time

        calculate_value = self.kp * error + self.ki * self.integ_error + self.kd * d_term

        self.last_error = error
        self.last_time = current_time

        return calculate_value
