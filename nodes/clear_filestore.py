#!/usr/bin/env python3

# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import os, sys
import tools.file_tools as file_tools
from tools.logging_tools import logger
from capturer.model import DAO, DBFM, CONFIG
 
# -----------------------------------------------------------
# This program is used to clear the filestore and delete all 
# bagfiles stored in the filestore folder 
if __name__ == '__main__':
    logger.info(f'Bagfile Capturer: Clear Filestore')
    filestore = CONFIG.get('capturer.filestore', None)
    if filestore is None:
        logger.warning(f'Error: the filestore (config "capturer.filestore") is not defined!')  
        sys.exit(1)
    if not os.path.isdir(filestore):
        logger.warning(f'Error: the filestore specified (config "capturer.filestore") is not a folder!')  
        sys.exit(2)
    # confirm with the user
    logger.info(f'Filestore location: {filestore}')
    if '-y' in sys.argv or '-yes' in sys.argv:
       answer = 'Y'
    else:
        logger.warning(f'Warning! All files in the filestore will be cleared!')  
        answer = input('Are you sure to delete all files and folder in the filestore (Y or N): ')

    if answer == 'Y':
        file_tools.remove_folder_content(filestore)
        logger.info(f'Bagfile Capturer: Clear Filestore Completed')
