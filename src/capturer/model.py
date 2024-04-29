# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

# general modules
import os, sys, threading, collections, time
from enum import Enum
import rospy
# project modules
import capturer.model_base
from capturer.model_base import *

# the global variable
CALLBACK_MANAGER = capturer.model_base.CallbackManager()

class CallbackTypes(Enum):
    TIMER = 0

class AssetFileManager():
    def __init__(self):
        self.user_home = os.path.expanduser('~') 
        self.capturere_home = rospy.get_param(f'/capturer_home', default=os.path.join(self.user_home, 'capturer_data'))

ASSET_FILE_MANAGER = AssetFileManager()
