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
from abc import ABC, abstractmethod

import rospy
from snips_nlu import SnipsNLUEngine
from snips_nlu.default_configs import CONFIG_EN


class KeywordParser(ABC):
    @abstractmethod
    def parse(self, text):
        pass


class NormalKeywordParser(KeywordParser):
    def __init__(self, configs: dict):
        self.configs = configs

    def parse(self, text):
        for meaning, configs in self.configs.items():
            require_keywords = configs['and']
            separately_keywords = configs['or']
            rospy.loginfo(f'{require_keywords}, {separately_keywords}')

            require_keywords_status = self._has_require_keywords(text, require_keywords)
            separately_keywords_status = self._has_separately_keywords(text, separately_keywords)

            if require_keywords_status and separately_keywords_status:
                response = configs['response']
                actions = configs['action']
                rospy.loginfo(f'Text \'{text}\' matched, Meaning: {meaning}, response: {response}')
                return meaning, response, actions
        else:
            rospy.logerr(f'Text \'{text}\' doesn\'t match anybody in the config file')
            return '', '', []

    @staticmethod
    def _input_text_processor(text):
        return text.strip().lower().split()

    def _has_require_keywords(self, text, require_keywords):
        input_text = self._input_text_processor(text)
        if len(require_keywords) == 0:
            return True

        for require_keyword in require_keywords:
            require_keyword = self._input_text_processor(require_keyword)[0]
            if require_keyword not in input_text:
                return False
        else:
            return True

    def _has_separately_keywords(self, text, separately_keywords):
        input_text = self._input_text_processor(text)
        if len(separately_keywords) == 0:
            return True

        for separately_keyword in separately_keywords:
            separately_keyword = self._input_text_processor(separately_keyword)[0]
            if separately_keyword in input_text:
                return True
        else:
            return False


class HeySnipsNLUParser(KeywordParser):
    def __init__(self, engine_path=None, dataset_path=None):
        if isinstance(engine_path, str):
            self.nlu_engine = SnipsNLUEngine.from_path(engine_path)
        elif isinstance(dataset_path, str):
            with open(dataset_path) as f:
                samples_dataset = json.load(f)

            self.nlu_engine = SnipsNLUEngine(config=CONFIG_EN)
            self.nlu_engine.fit(samples_dataset)

    def parse(self, text):
        parse_data = self.nlu_engine.parse(text)

        intent_data = parse_data['intent']
        user_intent = intent_data['intentName']
        intent_probability = intent_data['probability']

        parsed_slots = parse_data['slots']

        return user_intent, intent_probability, parsed_slots
