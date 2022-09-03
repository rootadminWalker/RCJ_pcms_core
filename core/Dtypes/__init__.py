# from .PoseProcess import *
from collections import namedtuple

try:
    from .SnipsProcess import *
except Exception:
    pass

from .boxProcess import *
from .Namespace import Namespace

# Subscribe Intent
SubscribeIntent = namedtuple('SubscribeIntent', ['callback', 'response'])
