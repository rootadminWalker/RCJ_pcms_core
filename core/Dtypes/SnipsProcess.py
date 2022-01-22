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
import json
from collections import namedtuple
from typing import Union, Tuple

ParseResult = namedtuple('ParseResult', 'engine_id intent intent_probability slots')

# Intent config
IntentConfig = namedtuple('IntentConfig', ['allow_preempt', 'max_re_ask', 'required_slots'])
RequiredSlot = namedtuple('SlotConfig', ['slot_name', 'confirm_response', 'confirm_intent'])

SlotRange = namedtuple('SlotRange', ['start', 'end'])
SlotValue = namedtuple('SlotValue', ['rawValue', 'value', 'kind'])
Slot = namedtuple('Slot', ['name', 'range', 'value', 'entity'])


class Slots:
    def __init__(self, raw_slots: dict):
        self.raw_slots = raw_slots
        self.slots = []
        for slot in self.raw_slots:
            slot_range = SlotRange(slot['range']['start'], slot['range']['end'])
            slot_value = SlotValue(slot['rawValue'], slot['value']['value'], slot['value']['kind'])
            self.slots.append(Slot(slot['slotName'], slot_range, slot_value, slot['entity']))

    def slot_exist(self, name: str) -> bool:
        for slot in self.slots:
            if name == slot.name:
                return True
        return False


class IntentConfigs:
    INTENT_DEFAULT_CONFIG = IntentConfig(allow_preempt=False, required_slots=[], max_re_ask=1)

    def __init__(self, config_path: str):
        with open(config_path) as f:
            intent_configs = json.load(f)
        for intent_name, json_data in intent_configs.items():
            required_slots = []
            for req_slot_name, req_configs in json_data['required_slots'].items():
                required_slot = RequiredSlot(
                    req_slot_name,
                    req_configs['confirm_response'],
                    req_configs['confirm_intent']
                )
                required_slots.append(required_slot)

            self.__dict__[intent_name] = IntentConfig(
                json_data['allow_preempt'],
                json_data['max_re-ask'],
                required_slots
            )

        self.__dict__['NotRecognized'] = IntentConfigs.INTENT_DEFAULT_CONFIG

    def slots_insufficient(self, target_intent: str, slots: Slots) -> Tuple[bool, Union[RequiredSlot, None]]:
        for required_slot in self.__dict__[target_intent].required_slots:
            if not slots.slot_exist(required_slot.slot_name):
                return True, required_slot
        return False, None

    def __getitem__(self, item):
        return self.__dict__[item]
