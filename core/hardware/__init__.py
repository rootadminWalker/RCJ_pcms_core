import warnings

from ..base_classes import Unit


class Hardware(Unit):
    def __init__(self):
        super(Hardware, self)._check_status()



from .Chassis import Chassis
from .ManipulatorController import ManipulatorController
from .SLAMController import SLAMController
from .FacialDisplayController import FacialDisplayController
from .Speaker import Speaker
try:
    from .YourTTS import YourTTS
except ImportError as e:
    warnings.warn(f'YourTTS missing package: {e}')
