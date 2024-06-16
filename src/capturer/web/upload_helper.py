# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import traceback
import base64, io
import numpy as np
import pandas as pd
# project modules
from tools import file_tools, db_tools, type_tools
from tools.logging_tools import logger
import tools.hash_tools as hash_tools
from capturer.model import ASSET_FILE_MANAGER, DAO

# extract data from an excel file in a http request form, which may be coming from a web upload, by first
# converting it into byte array and then call the main function
def process_uploaded_excel_file(contents, sheet_list=None, filename=None, last_modified=None):
    content_type, content_string = contents.split(',')
    decoded = base64.b64decode(content_string)
    return process_excel_file(decoded, sheet_list, filename, last_modified)
    
# extract data from an excel file in byte array form 
def process_excel_file(decoded, sheet_list=None, filename=None, last_modified=None):
    error_list = []
    processor = {'Schedule': process_schedule, 'Accounts': process_account, 'Rostopics': process_rostopics}
    k = ''
    try:
        suffix = file_tools.get_suffix(filename, include_period=False)
        if suffix in ['xls', 'xlsx']:
            df_dict = pd.read_excel(io.BytesIO(decoded), sheet_name=None)
            # goes through the worksheets in the excel file, and handle only those in the processor list and in sheet_list
            for k in df_dict.keys():
                if (sheet_list is None or k in sheet_list) and k in processor:
                    # call the respective function defined in the processor list according to the worksheet names
                    this_error = processor[k](df_dict[k]) 
                    if this_error is not None:
                        error_list.extend(this_error)
        else:
            error_list.append(f'The suffix of {filename} is not xls or xlsx')
    except Exception as e:
        logger.error(f'Error: {traceback.format_exc()}')
        error_list.append(f'error in reading excel file or worksheet {k}: {e}')
    return error_list

# extract data from the Schedule worksheet and upload it to the db
def process_schedule(pd_schedule:pd.DataFrame):
    error_list = []
    pd_schedule.columns = ['the_date', 'the_time', 'duration', 'folder', 'prefix']
    if pd_schedule.duration.dtype != np.int64 and pd_schedule.duration.dtype != np.float32:
       error_list.append('non-numeric cell found in the "duration" or column (maybe a hidden space?)')

    if pd_schedule.the_date.dtype != object and pd_schedule.the_time.dtype != object:
        error_list.append('non-string cell found in the date or the time column') 
    pd_schedule['status'] = DAO.ScheduleStates.PENDING.value
    pd_schedule['timestamp'] = pd_schedule.apply(lambda row: type_tools.datetime_to_timestamp(row['the_date'], row['the_time']), axis=1)
    pd_schedule['folder'] = pd_schedule['folder'].fillna('')
    pd_schedule['prefix'] = pd_schedule['prefix'].fillna('')
    pd_schedule.dropna(subset=['timestamp'], inplace=True)
    pd_schedule = pd_schedule.astype({'the_date': 'str', 'the_time': 'str', 'folder': 'str', 'duration': 'str',
                              'prefix': 'str', 'status': 'uint8'})
    # replace the tables of tile and tank
    with db_tools.create_connection(DAO.db_file) as conn:
        pd_schedule.to_sql('schedule', con=conn, if_exists='replace', index=False)
    return error_list

# extract data from the Accounts worksheet and upload it to the db
def process_account(pd_account:pd.DataFrame):
    error_list = []
    pd_account.columns = ['user', 'password', 'name', 'email']
    with db_tools.create_connection(DAO.db_file) as conn:
        DAO.clear_accounts()
        for i in pd_account.index:
            row = pd_account.iloc[i]
            salt, hashed = hash_tools.hash_new_password(row['password'])
            name = '' if row['name'] == None else row['name']
            email = '' if row['email'] == None else row['email']
            DAO.add_account(row['user'], hashed, name, email, salt)
    return error_list

# extract data from the Rostopics worksheet and upload it to the db
def process_rostopics(pd_rostopics:pd.DataFrame):
    error_list = []
    pd_rostopics.columns = ['topic_name']
    with db_tools.create_connection(DAO.db_file) as conn:
        DAO.clear_rostopics()
        for i in pd_rostopics.index:
            row = pd_rostopics.iloc[i]
            DAO.add_rostopic(row['topic_name'])
    return error_list