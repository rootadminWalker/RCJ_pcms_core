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

from typing import List, NamedTuple, Any

import hcl

# Snips engine parse result
from .Namespace import Namespace


class ParseResult(NamedTuple):
    engine_id: str = ''
    intent: str = None
    intent_probability: float = .0
    slots: List[dict] = []


class SlotValue(NamedTuple):
    rawValue: str = ''
    value: Any = None
    kind: str = ''


class Slot(NamedTuple):
    name: str = ''
    range: tuple = ()
    value: SlotValue = None
    entity: str = ''


class Slots:
    def __init__(self, raw_slots):
        self.raw_slots = raw_slots
        self.slots = []
        for slot in self.raw_slots:
            self.slots.append(self.convert_dict_to_slot(slot))

    @staticmethod
    def convert_dict_to_slot(slot: dict):
        slot_range = (slot['range']['start'], slot['range']['end'])
        slot_value = SlotValue(slot['rawValue'], slot['value']['value'], slot['value']['kind'])
        return Slot(slot['slotName'], slot_range, slot_value, slot['entity'])

    def update_slot(self, slot: dict):
        if (n := self.slot_idx(slot['slotName'])) > -1:
            self.raw_slots[n] = slot
            self.slots[n] = self.convert_dict_to_slot(slot)
        else:
            self.raw_slots.append(slot)
            self.slots.append(self.convert_dict_to_slot(slot))

    def update_slots(self, slots: List[dict]):
        for slot in slots:
            self.update_slot(slot)

    def update_but_ignore_exist_slots(self, slots: List[dict]):
        for slot in slots:
            if not self.slot_exist(slot['slotName']):
                self.update_slot(slot)

    def list_slot_names(self):
        slot_names = []
        for slot in self.slots:
            slot_names.append(slot.name)
        return slot_names

    def slot_idx(self, name):
        for idx, slot in enumerate(self.slots):
            if name == slot.name:
                return idx
        return -1

    def slot_exist(self, name):
        return self.slot_idx(name) > -1


class IntentConfigs:
    INTENT_DEFAULT_CONFIG = Namespace(
        required_slots=[],
        max_re_ask=1,
        confirm_intent='',
        confirm_responses={}
    )

    def __init__(self, config_path: str):
        with open(config_path) as f:
            intent_configs = hcl.load(f)
            self.intents = Namespace.dict_to_namespace(intent_configs['intent'])
            self.engines = []
            self.datasets = []
            if 'engines' in intent_configs:
                self.engines = intent_configs['engines']
            if 'datasets' in intent_configs:
                self.datasets = intent_configs['datasets']

    def find_missing_slots(self, target_intent: str, slots: Slots) -> List[Slots]:
        missing_slots_list = []
        if target_intent not in self.intents.__dict__:
            return []

        for required_slot in self.intents[target_intent].required_slots:
            if not slots.slot_exist(required_slot):
                missing_slots_list.append(required_slot)

        return missing_slots_list
