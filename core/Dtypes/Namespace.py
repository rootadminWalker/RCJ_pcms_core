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
from types import SimpleNamespace
from typing import NewType


class Namespace(SimpleNamespace):
    """
    Namespace that supports dictionary key getting
    """

    def __getitem__(self, name):
        return getattr(self, name)

    @classmethod
    def dict_to_namespace(cls, d: dict) -> NewType:
        """
        Converts a dictionary into a core.Dtypes.Namespace
        Args:
            d: the dictionary

        Returns:
            Converted namespace
        """
        x = Namespace()
        _ = [setattr(x, k, cls.dict_to_namespace(v)) if isinstance(v, dict) else setattr(x, k, v) for k, v in d.items()]
        return x
