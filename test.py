import time
from digi.xbee.devices import DigiMeshDevice
from struct import *
import copy
from ctypes import *
from info import info, packet127, packet128
from datetime import datetime






# int(math.log(msg.HL_FAILURE_FLAG,2))
# if msg.severity < 4:
# msg.severity += 14


