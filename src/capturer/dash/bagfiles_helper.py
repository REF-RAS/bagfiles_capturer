# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import os, shutil
from capturer.model import ASSET_FILE_MANAGER, DAO, CONFIG

def clear_bagfiles(older_than=None):
    bagfile_list = DAO.list_bagfiles(older_than)
    for bagfile in bagfile_list:
        try:
            os.remove(bagfile['full_path'])
            DAO.remove_bagfiles_by_id(bagfile['id'])
        except Exception as e:
            pass
        
