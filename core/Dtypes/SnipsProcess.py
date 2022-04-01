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

from typing import List

import hcl

# Snips engine parse result
from . import Namespace


class IntentConfigs:
    INTENT_DEFAULT_CONFIG = Namespace(
        allow_preempt=False,
        max_re_ask=1,
        confirm_intent='',
        required_slots=[]
    )

    def __init__(self, config_path: str):
        with open(config_path) as f:
            intent_configs = hcl.load(f)
            self.intents = Namespace.dict_to_namespace(intent_configs['intent'])
            self.confirm_intents = Namespace.dict_to_namespace(intent_configs['confirm_intent'])

    def find_missing_slots(self, target_intent: str, slots: Namespace) -> List[Namespace]:
        for required_slot in self.__dict__[target_intent].required_slots:
            if not slots.slot_exist(required_slot.slot_name):
                yield required_slot

    def __getitem__(self, item):
        return self.__dict__[item]
