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
from datetime import datetime
# project modules
import capturer.model_base as model_base
from capturer.database_dao import DBFileManager, CapturerDAO
from tools.yaml_tools import YamlConfig
from tools.logging_tools import logger

class CallbackTypes(Enum):
    TIMER = 0

# The states of the bagfile capturer
class SystemStates(Enum):
    READY = 0
    CAPTURING = 1
    PULL_CONFIG = 2

# The global variables to be imported by other modules
CALLBACK_MANAGER = model_base.CallbackManager()
CONFIG:YamlConfig = YamlConfig(os.path.join(os.path.dirname(__file__), '../../config/capturer_config.yaml'))
DBFM = DBFileManager()
DAO = CapturerDAO(CONFIG, DBFM.db_file)
STATE = model_base.StateManager(SystemStates.READY)

# The class modelling the file manager to store bagfiles
class AssetFileManager():
    def __init__(self):
        self.log_lock = threading.Lock()
        self.user_home = os.path.expanduser('~') 
        self.bagfiles_folder = CONFIG.get('capturer.filestore', '/home/qcr/Insync/Bagfiles')
        os.makedirs(self.bagfiles_folder, exist_ok=True)
        self.logfiles_folder = CONFIG.get('capturer.logfilestore', 'Logfiles')
        self.logfiles_folder = os.path.join(self.bagfiles_folder, self.logfiles_folder)
        os.makedirs(self.logfiles_folder, exist_ok=True)
        
    def get_output_bagfiles_folder(self, folder:str=None) -> str:
        """ Return the folder for storing the output bagfile, given the folder under the filestore

        :param folder: The folder under the bagfile filestore, defaults to None which refers to the root
        :return: The full path to the folder for the output bagfile
        """
        folder = '' if folder is None else folder.strip()
        if len(folder) == 0:
            return self.bagfiles_folder
        full_path =  os.path.join(self.bagfiles_folder, folder)
        os.makedirs(full_path, exist_ok=True)
        return full_path
    
    def record_event(self, message, no_logfile=False, no_screen=False) -> None:
        """ An utility for saving the message to a log file and as the same time to the logger

        :param message: The message to be recorded, type str
        """
        with self.log_lock:
            datestr = datetime.now().strftime('%Y%m%d')
            timestr = datetime.now().strftime('%Y/%m/%d %H:%M:%S')
            log_file_path = os.path.join(self.logfiles_folder, f'{datestr}.txt')
            message = timestr + ": " + message
            if not no_logfile:
                with open(log_file_path, 'a') as myfile:
                    myfile.write(message + '\n')
            if not no_screen:
                logger.info(message)

# The global variable to be imported by other modules
ASSET_FILE_MANAGER = AssetFileManager()


