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

from core.Dtypes import SubscribeIntent, Slots, IntentConfigs
from core.Nodes import Node
from core.tools import Speaker


def dummy_callback(intent, slots, raw_text, session):
    return


class ActionEvaluator(Node):
    def __init__(self):
        super(ActionEvaluator, self).__init__('intent_ac', anonymous=False)

        # Load the intent configs
        config_file = rospy.get_param(
            '~config_file',
            '/home/root_walker/workspace/ROS_projects/src/The_Essense_of_the_Grey_Region/config/ActionController/SnipsIntentConfigs/restaurant.json'
        )
        self.intent_configs = IntentConfigs(config_file)

        self.intent2callback = {}

        # Call the start and stop flow service
        self.__start_session = rospy.ServiceProxy('/intent_manager/start_session', Session)
        self.__continue_session = rospy.ServiceProxy('/intent_manager/continue_session', Session)
        self.__stop_session = rospy.ServiceProxy('/intent_manager/stop_session', Trigger)

        # Initialize the speaker
        self.speaker = Speaker()

        self.current_intent = ''

        # Initialize the action server
        self.action_controller_server = actionlib.SimpleActionServer(
            'intent_ac',
            IntentACControllerAction,
            execute_cb=self.message_cb,
            auto_start=False
        )
        self.action_controller_server.start()

        self.main()

    def subscribe_intent(self, intent, callback, response):
        self.intent2callback[intent] = SubscribeIntent(callback, response)

    def insufficient_callback(self, slots, required_slot):
        self.speaker.say_until_end(required_slot.confirm_response)
        if not self.on_session():
            self.start_session(
                next_intents=[self.current_intent],
                custom_data=json.dumps(slots.raw_slots)
            )
        else:
            self.continue_session(next_intents=[self.current_intent])

    @staticmethod
    def on_session():
        return rospy.get_param('/intent_manager/on_session')

    def message_cb(self, goal: IntentACControllerGoal):
        # TODO async the callback function for preempt checking
        # Parse the data
        self.current_intent = goal.intent
        slots = Slots(json.loads(goal.slots))
        raw_text = goal.raw_text
        session = None
        if self.on_session():
            session = goal.session

        # Execute the callbacks
        if self.current_intent not in self.intent2callback:
            rospy.logerr(f"Intent {self.current_intent}'s callback doesn't exist, doing nothing")
            callback = dummy_callback
        else:
            resp_and_callback = self.intent2callback[self.current_intent]
            self.speaker.say_until_end(resp_and_callback.response)
            callback = resp_and_callback.callback

        # Get intent configs from the config file
        try:
            intent_config = self.intent_configs[self.current_intent]
        except KeyError:
            intent_config = IntentConfigs.INTENT_DEFAULT_CONFIG

        # check if the slots are sufficient for executing
        slots_insufficient, required_slot = self.intent_configs.slots_insufficient(self.current_intent, slots)
        if slots_insufficient:
            self.insufficient_callback(slots, required_slot)
        else:
            try:
                self.stop_session()
            except RuntimeError:
                pass
            callback(self.current_intent, slots, raw_text, session)

        # Set the callback was executed successfully
        self.current_intent = ''
        result = IntentACControllerResult(True)
        self.action_controller_server.set_succeeded(result)

    def start_session(self, next_intents, custom_data=None):
        if self.on_session():
            raise RuntimeError('A session has started already')

        req = SessionRequest()
        req.session_data.started_intent = self.current_intent
        req.session_data.possible_next_intents = next_intents
        req.session_data.custom_data = json.dumps(custom_data)
        self.__start_session(req)

    def continue_session(self, next_intents, custom_data=None):
        if not self.on_session():
            raise RuntimeError("You can't continue a 'None' session")

        req = SessionRequest()
        req.session_data.possible_next_intents = next_intents
        if custom_data is not None:
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
