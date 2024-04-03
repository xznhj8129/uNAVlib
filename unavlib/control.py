import threading
from collections import deque
from itertools import cycle
import argparse
import tomllib

from . import mspy
from . import msp_ctrl
from . import msp_codes
from . import inav_modes

class unav_controller():
    def __init__(self, mode_config, device, baudrate=115200, trials=1, use_tcp=False, use_proxy=False):
        self.channels = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.pwm_range = [1000,2000]
        self.mode_range = [900,2100]
        self.mode_increments = 25
        self.pwm_midpoint = 1500
        self.msp_override = True
        self.mspy_min_time_between_writes = 1/100
        self.mspy_loglevel='INFO' 
        self.mspy_timeout=1/100
        self.mspy_trials=1
        self.mspy_logfilename='MSPy.log' 
        self.mspy_logfilemode='a'

if __main__:
    mode_config = configparser.ConfigParser()



                
