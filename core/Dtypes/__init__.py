# from .PoseProcess import *
from collections import namedtuple

from .SnipsProcess import *
from .boxProcess import *
from .Namespace import Namespace

# Subscribe Intent
SubscribeIntent = namedtuple('SubscribeIntent', ['callback', 'response'])
