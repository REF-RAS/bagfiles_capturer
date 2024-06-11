#!/usr/bin/env python3

# Copyright 2023 - Andrew Kwok Fai LUI, Centre for Robotics
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2023, The CGRAS Project'
__license__ = 'GPL'
__version__ = '0.0.1'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import os, datetime, shutil
from enum import Enum
from datetime import datetime as dt
import capturer.database_setup as database_setup
import tools.db_tools as db_tools
import tools.file_tools as file_tools
from tools.lock_tools import synchronized
from tools.logging_tools import logger
import rospy
 
# This class models the management and backup of db files and the folder that contains the files
class DBFileManager():
    DB_FOLDER_NAME = 'sqlite3'
    DB_FILENAME = 'capturer.db'
    def __init__(self):
        try:
            self.user_home = os.path.expanduser('~') 
            self.db_filename = DBFileManager.DB_FILENAME
            self.db_file = os.path.realpath(os.path.join(os.path.dirname(__file__), '../../db', self.db_filename))
            logger.info(f'DBFileManager sqlite db_file: {self.db_file}')
            self.db_parent_folder = file_tools.get_parent(self.db_file)
            # - test if the db_file exists, if not, create one
            if not os.path.isfile(self.db_file):
                database_setup.create_tables(self.db_file)
            else:
                # - making a backup of the database file if it has not been made this day
                self._make_daily_backup()
        except Exception as e:
            raise AssertionError(f'system database setup error: {e}')
        
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
                
# This class is the data access object for the bagfile capturer
class CapturerDAO():
    def __init__(self, config, db_file=None):
        if db_file is None:
            db_file = DBFM.db_file
        self.db_file = db_file
        self.CONFIG = config
        self.cached_started_scan = None # the cached current started scan

    ##### ---- VALIDATING THE DATABASE
    # return True if there is at least one tile, one tank, one station, and one pattern for the operation
    @synchronized
    def validate_db(self):
        with db_tools.create_connection(self.db_file) as conn:       
            c = conn.cursor() 
            result = c.execute('SELECT COUNT(*) FROM general_config').fetchone()
            if not result or result[0] == 0:
                return False         
        return True

    ##### ---- GENERAL CONFIG
    @synchronized
    def set_config_value(self, name, value):
        with db_tools.create_connection(self.db_file) as conn:
            c = conn.cursor()
            c.execute('REPLACE INTO general_config (name, value) VALUES (?, ?)', (name, value))
            conn.commit()
        return name   
    
    @synchronized
    def get_config_value(self, name, default=None):
        with db_tools.create_connection(self.db_file) as conn:       
            c = conn.cursor() 
            result = c.execute('SELECT value FROM general_config WHERE name = ?', (name,)).fetchone()
            if result is None:
                return default
            return result[0]
    
    ##### --- The states for the schedule table
    class ScheduleStates(Enum):
        PENDING = 0
        RECORDING = 1
        SUCCESS = 2
        FAILED = 3
        EXPIRED = 4
    
    @synchronized
    def list_schedule(self):
        sql = 'SELECT S.the_date, S.the_time, S.duration, S.timestamp, S.folder, S.prefix, S.status from schedule S ORDER BY S.timestamp ASC'
        return db_tools.query(self.db_file, sql)

    @synchronized
    def list_schedule_pending(self):
        sql = 'SELECT S.the_date, S.the_time, S.duration, S.timestamp, S.folder, S.prefix, S.status from schedule S WHERE S.status = ? ORDER BY S.timestamp ASC'
        return db_tools.query(self.db_file, sql, (self.ScheduleStates.PENDING.value,))
    
    @synchronized
    def update_schedule_status(self, timestamp:int, status:ScheduleStates):
        with db_tools.create_connection(self.db_file) as conn:
            c = conn.cursor()
            c.execute('UPDATE schedule SET status = ? WHERE timestamp = ?', (status.value, timestamp,))
            conn.commit()
    
    @synchronized
    def update_expired_schedule(self):    
        with db_tools.create_connection(self.db_file) as conn:
            c = conn.cursor()
            c.execute('UPDATE schedule SET status = ? WHERE DATETIME(timestamp, "unixepoch", "localtime") < DATETIME("now", "localtime", "-1 minutes") ', (self.ScheduleStates.EXPIRED.value,))
            conn.commit()        
    
    @synchronized
    def query_schedule(self, status:ScheduleStates):
        sql = 'SELECT * FROM schedule S WHERE S.status = ? AND DATETIME(S.timestamp, "unixepoch", "localtime") BETWEEN DATETIME("now", "localtime", "-1 minutes") AND DATETIME("now", "localtime") LIMIT 1'
        return db_tools.query_for_dict(self.db_file, sql, (status.value,))        

    ##### --- Table rostopics
    @synchronized
    def count_rostopics(self):
        return db_tools.count_rows(self.db_file, 'rostopics')
    
    @synchronized
    def list_rostopic_names(self):
        sql = 'SELECT topic_name FROM rostopics'
        topic_name_list =  db_tools.query_for_list(self.db_file, sql)     
        if len(topic_name_list) == 0:
            topic_name_list = self.add_default_rostopics()
        return topic_name_list

    @synchronized
    def clear_rostopics(self):
        sql = 'DELETE FROM rostopics'
        return db_tools.update(self.db_file, sql)

    @synchronized
    def add_rostopic(self, topic_name, topic_class=None):
        sql = 'INSERT INTO rostopics (topic_name, topic_class) VALUES (?, ?)'
        return db_tools.update(self.db_file, sql, (topic_name, topic_class,))
    
    def add_default_rostopics(self):
        topic_name_list = self.CONFIG.get('capturer.rostopics.default')
        for topic_name in topic_name_list:
            self.add_rostopic(topic_name)
        return topic_name_list
    
    #### --- Table bagfiles
    @synchronized
    def add_bagfiles(self, timestamp, duration, folder, file_name, full_path, file_size):
        with db_tools.create_connection(self.db_file) as conn:
            c = conn.cursor()
            c.execute('INSERT INTO bagfiles (timestamp, duration, folder, name, full_path, file_size) VALUES (?, ?, ?, ?, ?, ?)', 
                    (timestamp, duration, folder, file_name, full_path, file_size))
            conn.commit()
            bagfile_id = c.lastrowid
        return bagfile_id
    
    @synchronized
    def count_bagfiles(self):
        return db_tools.count_rows(self.db_file, 'bagfiles')

    @synchronized
    def list_latest_bagfiles(self, limit=10):
        sql = 'SELECT * FROM bagfiles ORDER BY timestamp DESC LIMIT ?'
        return db_tools.query(self.db_file, sql, (limit,))
    
    @synchronized
    def list_bagfiles(self, older_than=None):
        if older_than == None:
            sql = 'SELECT * FROM bagfiles B ORDER BY B.timestamp DESC'
            return db_tools.query_as_list_of_dicts(self.db_file, sql)            
        else:
            sql = 'SELECT * FROM bagfiles B WHERE DATETIME(B.timestamp, "unixepoch", "localtime") < DATETIME("now", "localtime", ?) ORDER BY B.timestamp DESC'
            return db_tools.query_as_list_of_dicts(self.db_file, sql, (older_than,))    

    @synchronized
    def remove_bagfiles_by_id(self, id):
        sql = 'DELETE FROM bagfiles WHERE id = ?'
        return db_tools.update(self.db_file, sql, (id,))

    @synchronized
    def count_accounts(self):
        return db_tools.count_rows(self.db_file, 'accounts')
    
    @synchronized
    def query_account(self, user) -> dict:
        sql = 'SELECT * FROM accounts WHERE user = ?'
        return db_tools.query_for_dict(self.db_file, sql, (user,))
    
    @synchronized
    def clear_accounts(self):
        sql = 'DELETE FROM accounts'
        return db_tools.update(self.db_file, sql)
    
    def add_account(self, user, password, name, email, salt):
        sql = 'INSERT INTO accounts (user, password, name, email, misc) VALUES (?, ?, ?, ?, ?)'
        return db_tools.update(self.db_file, sql, (user, password, name, email, salt,))

# ------------------------------------------------
# The main program for testing the clearing
# of database tables and creating them
if __name__ == '__main__':
    DBFM = DBFileManager()
    DAO = CapturerDAO(DBFM.db_file)
    database_setup.drop_tables(DAO.db_file)   
    errors = database_setup.create_tables(DAO.db_file)
    if errors is not None:
        logger.error(f'Error in create tabless: {errors}')
