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

import actionlib
import rospy
from home_robot_msgs.msg import IntentACControllerResult, IntentACControllerGoal, IntentACControllerAction, VoiceSession
from home_robot_msgs.srv import StartFlowRequest, StartSession
from std_srvs.srv import Trigger

from core.Nodes import Node
from core.tools import Speaker


def dummy_callback(intent, slots, raw_text, flowed_intents):
    return


class ActionEvaluator(Node):
    def __init__(self):
        super(ActionEvaluator, self).__init__('intent_ac', anonymous=False)
        # The user must initialize the callback map, or else the program won't start
        if 'intent2callback' not in self.__dict__:
            raise AttributeError('You must define the callbacks corresponding to the intents')

        # Call the start and stop flow service
        self.__start_flow = rospy.ServiceProxy('/intent_manager/start_session', StartSession)
        self.stop_flow = rospy.ServiceProxy('/intent_manager/stop_session', Trigger)

        # Initialize the speaker
        self.speaker = Speaker()

        # Initialize the action server
        self.action_controller_server = actionlib.SimpleActionServer(
            'intent_ac',
            IntentACControllerAction,
            execute_cb=self.message_cb,
            auto_start=False
        )
        self.action_controller_server.start()

    def message_cb(self, goal: IntentACControllerGoal):
        # TODO async the callback function for preempt checking
        # Parse the data
        intent = goal.intent
        slots = json.loads(goal.slots)
        raw_text = goal.raw_text
        flowed_intents = goal.flowed_intents

        # Execute the callbacks
        if intent not in self.intent2callback:
            rospy.logerr(f"Intent {intent}'s callback doesn't exist, doing nothing")
            callback = dummy_callback
        else:
            callback = self.intent2callback[intent]

        callback(intent, slots, raw_text, flowed_intents)

        # Set the callback was executed successfully
        result = IntentACControllerResult(True)
        self.action_controller_server.set_succeeded(result)

    def start_session(self, next_intents):
        req = StartFlowRequest()
        req.next_intents = next_intents
        result = self.__start_flow(req)
        if not result:
            raise ReferenceError('A flow has started already')

    def main(self):
        pass

    def reset(self):
        pass
