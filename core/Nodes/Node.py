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
from typing import Any
from rich.console import Console

import rospy

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

    def __init__(self, name, anonymous=False, default_state='', node_group='~'):
        """
        An abstract class for initializing into a ROS node, since ROS uses callbacks and hermes protocol,
        OOP will be better for variables to cross between different borders of the program
        Args:
            name: Name of the node
            anonymous: Generate a random ID after the node name or not
            default_state: The default state of the node, if you want a state machine
            node_group: A group state header of multiple node
        """
        with console.status(f"[magenta] Initializing node /{name}") as status:
            self.name = name
            rospy.init_node(name, anonymous=anonymous)
            console.log("Initialized ROS Node")
            self.rate = rospy.Rate(Node.ROS_RATE)
            console.log(f"Initialized ROS rate as {Node.ROS_RATE}")

            self.__node_group = node_group
            self.__state = default_state
            rospy.set_param(f'{self.__node_group}next_state', self.__state)

        console.print(f"[bold green] Node /{name} initialized sucessfully")

    @abstractmethod
    def main(self):
        """
        Override this method to put your main loop inside it
        Returns:

        """
        pass

    @abstractmethod
    def reset(self):
        """
        Override this method will reset every values in the Node
        """
        pass

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

    def set_state(self, state: Any):
        """
        Set the coming next state
        Args:
            state: Must be a base type

        Returns:

        """
        rospy.set_param(f'{self.__node_group}next_state', state)

    def next_state_to_param(self):
        """
        Call this to set the cached state (next_state parameter)
        Returns:

        """
        self.__state = rospy.get_param(f'{self.__node_group}next_state')

    def get_state(self) -> Any:
        """
        Get the cached state
        Returns: A base type

        """
        return self.__state
