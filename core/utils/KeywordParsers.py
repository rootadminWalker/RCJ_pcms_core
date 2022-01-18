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

from core.Dtypes import ParseResult, Slots


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
    INTENT_MAX_PROBABILITY = 0.42

    def __init__(self, engine_configs=None, dataset_configs=None):
        if dataset_configs is None:
            dataset_configs = {}
        if engine_configs is None:
            engine_configs = {}

        self.nlu_engines = {}
        for engine_id, engine_path in engine_configs.items():
            nlu_engine = SnipsNLUEngine.from_path(engine_path)
            self.nlu_engines[engine_id] = nlu_engine

        for dataset_id, dataset_path in dataset_configs.items():
            with open(dataset_path) as f:
                samples_dataset = json.load(f)

            nlu_engine = SnipsNLUEngine(config=CONFIG_EN)
            nlu_engine.fit(samples_dataset)
            self.nlu_engines[dataset_id] = nlu_engine

        self.__sort_keyfunc = lambda d: d.intent_probability

    def parse(self, text, key=None):
        if key is None:
            key = self.__intent_condition_valid

        full_data = list(self.parse_full_data(text))
        full_data.sort(key=self.__sort_keyfunc, reverse=True)

        if key(full_data[0]):
            return full_data[0]

        return ParseResult('', None, 0.0, Slots([]))

    @staticmethod
    def __intent_condition_valid(parse_data: ParseResult):
        return parse_data.intent is not None and \
               parse_data.intent_probability >= HeySnipsNLUParser.INTENT_MAX_PROBABILITY

    def parse_with_engine_id(self, text, engine_id):
        parse_data = self.nlu_engines[engine_id].parse(text)

        intent_data = parse_data['intent']
        user_intent = intent_data['intentName']
        intent_probability = intent_data['probability']

        parsed_slots = parse_data['slots']
        return ParseResult(engine_id, user_intent, intent_probability, parsed_slots)

    def parse_full_data(self, text):
        for engine_id in self.nlu_engines.keys():
            _, user_intent, intent_probability, parsed_slots = self.parse_with_engine_id(text, engine_id)
            yield ParseResult(engine_id, user_intent, intent_probability, Slots(parsed_slots))
