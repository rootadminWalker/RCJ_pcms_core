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
import random
from typing import Union

import actionlib
import rospy
from home_robot_msgs.msg import IntentACControllerResult, IntentACControllerGoal, IntentACControllerAction
from home_robot_msgs.srv import Session, SessionRequest, ContinueSess, ContinueSessRequest
from std_srvs.srv import Trigger

from core.Dtypes import IntentConfigs, SubscribeIntent, Slots
from core.Nodes import Controller, Node
from core.hardware import Speaker


def dummy_callback(intent, slots, raw_text, session):
    return


class ActionEvaluator(Node):
    def __init__(self):
        super(ActionEvaluator, self).__init__('intent_ac', anonymous=False)

        # Call the start and stop flow service
        self.__start_session = rospy.ServiceProxy('/intent_manager/start_session', Session)
        self.__continue_session = rospy.ServiceProxy('/intent_manager/continue_session', ContinueSess)
        self.__stop_session = rospy.ServiceProxy('/intent_manager/stop_session', Trigger)

        # Initialize the speaker
        self.speaker = Speaker()

        # Intent to callbacks
        self.intent2callback = {}
        self.subscribe_intent('NotRecognized', dummy_callback, 'Not recognized')
        self.subscribe_intent('Confirm', self.confirm_callback, '')

        # The current intent
        self.current_intent = ''

        # Initialize the action server
        self.action_controller_server = actionlib.SimpleActionServer(
            'intent_ac',
            IntentACControllerAction,
            execute_cb=self.message_cb,
            auto_start=False
        )
        self.action_controller_server.start()

    @staticmethod
    def on_session(session):
        return session.id != ''

    def start_session(self, custom_data, possible_next_intents):
        session_req = SessionRequest()
        session_req.session_data.started_intent = self.current_intent
        session_req.session_data.custom_data = json.dumps(custom_data)
        session_req.session_data.possible_next_intents = possible_next_intents
        self.__start_session(session_req)

    def continue_session(self, custom_data, possible_next_intents):
        continue_req = ContinueSessRequest()
        continue_req.custom_data = json.dumps(custom_data)
        continue_req.possible_next_intents = possible_next_intents
        self.__continue_session(continue_req)

    def stop_session(self):
        self.__stop_session()

    def subscribe_intent(self, intent: str, callback=dummy_callback, response: Union[list, str] = '') -> None:
        """
        Assign a callback to an intent when it comes
        Args:
            intent: The name of the intent
            callback: The function that handles the intent
            response: The response of this intent

        Returns:

        """
        self.intent2callback[intent] = SubscribeIntent(callback, response)

    def message_cb(self, goal: IntentACControllerGoal):
        intent = goal.intent
        self.current_intent = intent
        slots = Slots(json.loads(goal.slots))
        raw_text = goal.raw_text
        session = goal.session
        if self.on_session(session):
            session.custom_data = json.loads(session.custom_data)

        if intent in self.intent2callback:
            intent_data = self.intent2callback[intent]
            intent_data.callback(intent, slots, raw_text, session)
            response = intent_data.response
            if isinstance(response, list):
                response = random.choice(response)
            self.speaker.say_until_end(response)

        self.action_controller_server.set_succeeded(IntentACControllerResult(True))

    def confirm_callback(self, intent, slots, raw_text, session):
        print(session)
        confirm_resp = session.custom_data['confirm_resp']
        self.speaker.say_until_end(confirm_resp)

    def start(self):
        self.main()

    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def reset(self):
        pass
