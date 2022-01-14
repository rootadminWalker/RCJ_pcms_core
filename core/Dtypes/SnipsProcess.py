#!/usr/bin/env python3

"""
MIT License

Copyright (c) 2019 rootadminWalker

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
z6
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""

# Snips engine parse result
from collections import namedtuple

ParseResult = namedtuple('ParseResult', 'engine_id user_intent intent_probability parsed_slots')

# Intent config
IntentConfig = namedtuple('IntentConfig', ['allow_preempt', 'required_slots'])

SlotRange = namedtuple('SlotRange', ['start', 'end'])
SlotValue = namedtuple('SlotValue', ['rawValue', 'value', 'kind'])
Slot = namedtuple('Slot', ['name', 'range', 'value', 'entity'])


class Slots:
    def __init__(self, slots_dict):
        self.raw_slots = slots_dict
        self.slots = []
        for slot in self.raw_slots:
            slot_range = SlotRange(slot['range']['start'], slot['range']['end'])
            slot_value = SlotValue(slot['rawValue'], slot['value']['value'], slot['value']['kind'])

            self.slots.append(Slot(slot['slotName'], slot_range, slot_value, slot['entity']))

    def slot_exist(self, name):
        for slot in self.slots:
            if name == slot.name:
                return True
        return False
