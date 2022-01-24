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


def dummy_callback(intent, slots, raw_text, session, missed_slots):
    return


class ActionEvaluator(Node):
    def __init__(self):
        super(ActionEvaluator, self).__init__(
            'intent_ac',
            anonymous=False,
            default_state='NORMAL',
            node_group='/intent/'
        )

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

        self.main()

    @staticmethod
    def on_session() -> bool:
        """
        Check if there is a established session
        Returns: True or False

        """
        return rospy.get_param('/intent_manager/on_session')

    def subscribe_intent(self, intent: str, callback, response: str) -> None:
        """
        Assign a callback to an intent when it comes
        Args:
            intent: The name of the intent
            callback: The function that handles the intent
            response: The response of this intent

        Returns:

        """
        self.intent2callback[intent] = SubscribeIntent(callback, response)

    def insufficient_callback(self, slots, required_slot):
        self.speaker.say_until_end(required_slot.confirm_response)
        if not self.on_session():
            self.__start_insufficient_session(
                next_intents=[],
                custom_data=json.dumps(slots.raw_slots)
            )
        else:
            self.__continue_insufficient_session(next_intents=[])

    def message_cb(self, goal: IntentACControllerGoal):
        # TODO async the callback function for preempt checking
        # Parse the data
        self.current_intent = goal.intent
        slots = Slots(json.loads(goal.slots))
        raw_text = goal.raw_text
        session = None
        if self.on_session():
            session = goal.session

        # check if the slots are sufficient for executing
        missed_slots = self.intent_configs.find_missing_slots(self.current_intent, slots)

        # Execute the callbacks
        if self.current_intent not in self.intent2callback:
            rospy.logerr(f"Intent {self.current_intent}'s callback doesn't exist, doing nothing")
            callback = dummy_callback
        else:
            resp_and_callback = self.intent2callback[self.current_intent]
            self.speaker.say_until_end(resp_and_callback.response)
            callback = resp_and_callback.callback

        callback(self.current_intent, slots, raw_text, session, missed_slots)

        self.next_state_to_param()

        # Set the callback was executed successfully
        self.current_intent = ''
        result = IntentACControllerResult(True)
        self.action_controller_server.set_succeeded(result)

    def __start_session_partial(self, session_type, next_intents, custom_data=None, max_rounds=1):
        if self.on_session():
            raise RuntimeError('A session has started already')

        req = SessionRequest()
        req.session_data.started_intent = self.current_intent
        req.session_data.session_type = session_type
        req.session_data.possible_next_intents = next_intents
        req.session_data.custom_data = json.dumps(custom_data)
        req.session_data.max_rounds = max_rounds
        self.__start_session(req)

    def __continue_session_partial(self, next_intents, custom_data=None):
        if not self.on_session():
            raise RuntimeError("You can't continue a 'None' session")

        req = SessionRequest()
        req.session_data.possible_next_intents = next_intents
        if custom_data is not None:
            req.session_data.custom_data = json.dumps(custom_data)
        req.session_data.max_rounds += 1
        self.__continue_session(req)

    def __start_insufficient_session(self, next_intents, custom_data=None, max_rounds=3):
        self.set_state('INSUFFICIENT')
        self.__start_session_partial('INSUFFICIENT', next_intents, custom_data, max_rounds)

    def __continue_insufficient_session(self, next_intents, custom_data=None):
        self.set_state('INSUFFICIENT')
        self.__continue_session_partial(next_intents, custom_data)

    def start_session(self, next_intents, custom_data=None):
        """
        Establish a new session, which last for one round.
        Args:
            next_intents: A threshold for the coming intents, can be None if you allow anybody
            custom_data: Data to pass through the whole session

        Returns:

        """
        self.set_state('ON_SESSION')
        self.__start_session_partial('NORMAL', next_intents, custom_data, 1)

    def continue_session(self, next_intents, custom_data=None):
        """
        Continue a establish session if you don't want it to perish
        Args:
            next_intents: A threshold for the coming intents, can be None if you allow anybody
            custom_data: Data to pass through the whole session

        Returns:

        """
        self.set_state('ON_SESSION')
        self.__continue_session_partial(next_intents, custom_data)

    def stop_session(self):
        """
        Stop an established session
        Returns:

        """
        if not self.on_session():
            raise RuntimeError("There isn't a session started")

        self.set_state('NORMAL')
        self.__stop_session()

    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def reset(self):
        pass
