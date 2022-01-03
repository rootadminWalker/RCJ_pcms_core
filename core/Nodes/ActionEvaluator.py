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
from home_robot_msgs.msg import IntentACControllerResult, IntentACControllerGoal, IntentACControllerAction
from home_robot_msgs.srv import Session, SessionRequest
from std_srvs.srv import Trigger

from core.Nodes import Node
from core.tools import Speaker


def dummy_callback(intent, slots, raw_text, session):
    return


class ActionEvaluator(Node):
    def __init__(self):
        super(ActionEvaluator, self).__init__('intent_ac', anonymous=False)
        # The user must initialize the callback map, or else the program won't start
        if 'intent2callback' not in self.__dict__:
            raise AttributeError('You must define the callbacks corresponding to the intents')

        # Call the start and stop flow service
        self.__start_session = rospy.ServiceProxy('/intent_manager/start_session', Session)
        self.__continue_session = rospy.ServiceProxy('/intent_manager/continue_session', Session)
        self.__stop_session = rospy.ServiceProxy('/intent_manager/stop_session', Trigger)

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

        self.main()

    @staticmethod
    def on_session():
        return rospy.get_param('/intent_manager/on_session')

    def message_cb(self, goal: IntentACControllerGoal):
        # TODO async the callback function for preempt checking
        # Parse the data
        intent = goal.intent
        slots = json.loads(goal.slots)
        raw_text = goal.raw_text
        session = None
        if self.on_session():
            session = goal.session

        # Execute the callbacks
        if intent not in self.intent2callback:
            rospy.logerr(f"Intent {intent}'s callback doesn't exist, doing nothing")
            callback = dummy_callback
        else:
            callback = self.intent2callback[intent]

        callback(intent, slots, raw_text, session)

        # Set the callback was executed successfully
        result = IntentACControllerResult(True)
        self.action_controller_server.set_succeeded(result)

    def start_session(self, next_intents, custom_data=None):
        if self.on_session():
            raise RuntimeError('A session has started already')

        req = SessionRequest()
        req.session_data.possible_next_intents = next_intents
        req.session_data.custom_data = json.dumps(custom_data)
        self.__start_session(req)

    def continue_session(self, next_intents, custom_data=None):
        if not self.on_session():
            raise RuntimeError("You can't continue a 'None' session")

        req = SessionRequest()
        req.session_data.possible_next_intents = next_intents
        req.session_data.custom_data = json.dumps(custom_data)
        self.__continue_session(req)

    def stop_session(self):
        if not self.on_session():
            raise RuntimeError("There isn't a session started")
        self.__stop_session()

    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def reset(self):
        pass
