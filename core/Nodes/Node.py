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
import warnings
from abc import ABC, abstractmethod
from typing import Any, Callable, List

import rospy
from rich.console import Console
from std_srvs.srv import Trigger, TriggerResponse

console = Console()


class NodeProgram:
    # TODO: Deprecate this after ActionController was merged
    def __init__(self, node_id):
        warnings.warn('This will be deprecated after ActionController was merged with ActionControllerNode',
                      DeprecationWarning)
        self.id = node_id

    @abstractmethod
    def serialize_output(self) -> Any:
        pass


class Node(ABC):
    ROS_RATE = 35

    def __init__(self, name, anonymous=False):
        """
        An abstract class for initializing into a ROS node, since ROS uses callbacks and hermes protocol,
        OOP will be better for variables to cross between different borders of the program
        Args:
            name: Name of the node
            anonymous: Generate a random ID after the node name or not
        """
        with console.status(f"[magenta] Initializing node /{name}") as status:
            self.name = name
            rospy.init_node(name, anonymous=anonymous)
            console.log("Initialized ROS Node")
            self.rate = rospy.Rate(Node.ROS_RATE)
            console.log(f"Initialized ROS rate as {Node.ROS_RATE}")
            rospy.Service('~reset', Trigger, self.reset)
            console.log(f"Initialized reset service")

        console.print(f"[bold green] Node /{name} initialized sucessfully")

    @abstractmethod
    def main(self):
        """
        Override this method to put your main loop inside it
        Returns:

        """
        pass

    @abstractmethod
    def reset(self) -> TriggerResponse:
        """
        Override this method to reset your own node by ~reset service
        """
        return TriggerResponse()

    @staticmethod
    def spin():
        """
        Call rospy.spin to spin the node
        """
        rospy.spin()

    @staticmethod
    def wait_for_msg(topic, data_class):
        """
        You can use this method for waiting a msg with info coming out

        Args:
            topic: The topic you want to wait for
            data_class:

        Returns: None

        """
        rospy.loginfo(f'Waiting response from {topic}')
        rospy.wait_for_message(topic, data_class)
        rospy.loginfo(f'{topic}: Ok')


class Controller(Node):
    def __init__(
            self,
            name: str,
            anonymous: bool = False,
            default_state: str = '',
            state_param_group: str = '~',
            states: List[str] = []
    ):
        """
        Controller node with state controlling
        Args:
            name: Name of the controller (Will be passed to rospy.init_node)
            anonymous: Will be passed to rospy.init_node as anonymous parameter
            default_state: The default state of the controller
            state_param_group: The state parameter's "first name". Used when multiple controllers share states
            states: List of states available (required)
        """
        super(Controller, self).__init__(name, anonymous)
        self.default_state = default_state
        self.state_param_group = state_param_group
        self.states = states
        assert len(self.states) != 0

        self.state_param = f'{self.state_param_group}/state'

        rospy.set_param(self.state_param, self.default_state)

        # The dictionary of states and corresponding listeners
        self.state_to_callback = {}

    def add_state_listener(self, callback: Callable, state: str):
        """
        Add a callback listener to a state. Will be called when self.execute_states is triggered
        Args:
            callback: The callback that listen to the specified state
            state: The state that correspond to the callback

        Returns:

        """
        assert state in self.states
        self.state_to_callback[state] = callback

    def get_state(self) -> Any:
        """
        Get the current state
        Returns:
            Current state
        """
        return rospy.get_param(self.state_param)

    def set_state(self, state: Any):
        """
        Sets the state to new a new state
        **WARNING** This will IMMEDIATELY replace the old state once you call it,
                    use it wisely
        Args:
            state: Anything represents a state and can be taken by rospy.set_param

        Returns:

        """
        rospy.set_param(self.state_param, state)

    def execute_states(self, *args, **kwargs):
        """
        This executes the listeners according to the state when this method is called
        *args and **kwargs will be passed to the callbacks directly

        Returns:

        """
        current_state = self.get_state()
        self.state_to_callback[current_state](*args, **kwargs)

    @abstractmethod
    def main(self):
        """
        Your node's main loop will evaluate here
        Returns:

        """

    @abstractmethod
    def reset(self):
        """
        You can reset your node's variables, states, etc.
        Default will reset the state to self.default_state
        Returns:

        """
        self.set_state(self.default_state)
