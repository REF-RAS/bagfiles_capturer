#!/usr/bin/env python3

# Copyright 2023 - Andrew Kwok Fai LUI, Centre for Robotics
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2023, The CGRAS Project'
__license__ = 'GPL'
__version__ = '0.0.1'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

# ----- the common modules
import os, json, random, copy, glob, collections, sys, time, datetime, shutil
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from datetime import datetime as dt
import sqlite3
import capturer.database_setup
import tools.db_tools as db_tools
import tools.file_tools as file_tools
from tools.yaml_tools import YamlConfig
import rospy
 
class DBFileManager():
    DB_FOLDER_NAME = 'sqlite3'
    DB_FILENAME = 'cgras_imaging.db'
    def __init__(self):
        try:
            self.cgras_home = rospy.get_param(f'/cgras/cgras_home', '/home/qcr/cgras_data')
            print(f'cgras data home: {self.cgras_home}')
            self.db_filename = ImagingDBFileManager.DB_FILENAME
            self.db_file = os.path.join(self.cgras_home, ImagingDBFileManager.DB_FOLDER_NAME, self.db_filename) 
            self.db_parent_folder = file_tools.get_parent(self.db_file)
            # - test if the db_file exists, if not, create one
            if not os.path.isfile(self.db_file):
                imaging.imaging_database_setup.create_tables(self.db_file)
            else:
                # - making a backup of the database file if it has not been made this day
                self._make_daily_backup()
        except Exception as e:
            raise AssertionError(f'system setup error: check "/cgras_home" in the launch file')
        
    def _make_daily_backup(self):
        today = datetime.date.today()
        date_str = today.strftime("%Y-%m-%d")
        daily_backup_filename = f'{date_str}_{self.db_filename}'
        daily_backup_path = os.path.join(self.db_parent_folder, daily_backup_filename)
        if os.path.isfile(daily_backup_path):
            return
        shutil.copyfile(self.db_file, daily_backup_path)
    
    def make_backup(self, label='S', use_move=False):
        now = datetime.now()
        time_str = now.strftime("%Y-%m-%d-%H-%M-%S")
        daily_backup_filename = f'{time_str}-{label}_{self.db_filename}' 
        daily_backup_path = os.path.join(self.db_parent_folder, daily_backup_filename)
        if os.path.isfile(daily_backup_path):
            return
        if use_move:
            shutil.move(self.db_file, daily_backup_path)        
        else:
            shutil.copyfile(self.db_file, daily_backup_path)               
        
    def get_backup_files(self):
        file_info_dict = dict()
        for f in os.listdir(self.db_parent_folder):
            fpath = os.path.join(self.db_parent_folder, f)
            if not os.path.isfile(fpath) :
                continue
            if f == self.db_filename or self.db_filename not in f:
                continue 
            backup_date, _ = f.split('_', 1)
            file_info_dict[backup_date] = f
        return file_info_dict
    
    def restore_backup(self, backup_filename):
        backup_filepath = os.path.join(self.db_parent_folder, backup_filename)
        if not os.path.isfile(backup_filepath) :
            return
        # make a backup of the existing db_file
        self.make_backup(label='RES', use_move=True)
        # make a copy of the backup file and save as db_file
        shutil.copyfile(backup_filepath, self.db_file)
                

class CapturerDAO():
    def __init__(self, db_file=None):
        if db_file is None:
            db_file = DBFM.db_file
        self.db_file = db_file
        self.cached_started_scan = None # the cached current started scan

    ##### ---- VALIDATING THE DATABASE
    # return True if there is at least one tile, one tank, one station, and one pattern for the operation
    def validate_db(self):
        with db_tools.create_connection(self.db_file) as conn:       
            c = conn.cursor() 
            result = c.execute('SELECT COUNT(*) FROM tile').fetchone()
            if not result or result[0] == 0:
                return False
            result = c.execute('SELECT COUNT(*) FROM tile_location').fetchone()
            if not result or result[0] == 0:
                return False                   
            result = c.execute('SELECT COUNT(*) FROM station').fetchone()
            if not result or result[0] == 0:
                return False            
            result = c.execute('SELECT COUNT(*) FROM tank').fetchone()
            if not result or result[0] == 0:
                return False    
            result = c.execute('SELECT COUNT(*) FROM scan_pattern').fetchone()
            if not result or result[0] == 0:
                return False             
        return True

    ##### ---- GENERAL CONFIG
    def set_config_value(self, name, value):
        with db_tools.create_connection(self.db_file) as conn:
            c = conn.cursor()
            c.execute('REPLACE INTO general_config (name, value) VALUES (?, ?)', (name, value))
            conn.commit()
        return name   
    
    def get_config_value(self, name, default=None):
        with db_tools.create_connection(self.db_file) as conn:       
            c = conn.cursor() 
            result = c.execute('SELECT value FROM general_config WHERE name = ?', (name,)).fetchone()
            if result is None:
                return default
            return result[0]       

# ----- create a global variable for ease of access
DBFM = ImagingDBFileManager()
DAO = CapturerDAO()

if __name__ == '__main__':
    imaging.imaging_database_setup.drop_tables(DAO.db_file)   
    imaging.imaging_database_setup.create_tables(DAO.db_file)

