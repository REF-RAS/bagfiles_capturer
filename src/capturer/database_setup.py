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

import os, json, random, copy, glob, collections, sys, yaml
import tools.db_tools as db_tools
import tools.file_tools as file_tools
from tools.logging_tools import logger

# The scripts for creating tables for the database
CREATE_TABLES_SQL = [""" 
CREATE TABLE IF NOT EXISTS general_config (
    name text PRIMARY KEY,
    value text
);
""",
"""
CREATE TABLE IF NOT EXISTS schedule (
    the_date text,
    the_time text,
    duration text,
    timestamp integer,
    folder text,
    prefix text,
    status integer
);
""",
"""
CREATE TABLE IF NOT EXISTS rostopics (
    topic_name text,
    topic_class text DEFAULT ''
);
""",
"""
CREATE TABLE IF NOT EXISTS bagfiles (
    id integer PRIMARY KEY,
    timestamp integer,
    duration text,
    folder text,
    name text,
    full_path text,
    file_size integer
);
""",
"""
CREATE TABLE IF NOT EXISTS accounts (
    user text,
    password blob,
    name text,
    email text,
    misc blob 
);
"""
]

# The script for dropping tables in the database
DROP_TABLES_SQL = """
DROP TABLE IF EXISTS general_config;
DROP TABLE IF EXISTS schedule;
DROP TABLE IF EXISTS rostopics;
DROP TABLE IF EXISTS bagfiles;
"""

# The script for clearing the data in the tables in the database
CLEAR_TABLES_SQL = """
DELETE FROM general_config;
DELETE FROM schedule;
DELETE FROM rostopics;
DELETE FROM bagfiles;
"""

def drop_tables(db_file):
    """ A convenient function for dropping all tables by executing the given script

    :param db_file: The path to the sqlite db file
    :return: The error as a string if any
    """
    os.makedirs(file_tools.get_parent(db_file), exist_ok=True)
    return db_tools.update_with_script_no_exception(db_file, DROP_TABLES_SQL)

def create_tables(db_file):
    """ A convenient function for creating all database tables for the bagfiles capturer

    :param db_file: The path to the sqlite db file
    :return: The error as a string if any
    """
    count = 0
    error_list = []
    os.makedirs(file_tools.get_parent(db_file), exist_ok=True)
    for create_sql in CREATE_TABLES_SQL:
        result = db_tools.update_with_script_no_exception(db_file, create_sql)
        if result is None:
            logger.info(f'{__file__} (create_tables): successful in {create_sql}')
            count += 1
        else:
            error_list.append(result)
            logger.warning(f'{__file__} (create_tables): error {result} in {create_sql}')
    if len(error_list) == 0:
        return None
    return '\n'.join(error_list)

def clear_tables(db_file):
    """ A convenient function for clearing the data of all the tables in the database

    :param db_file: The path to the sqlite db file
    :return: The error as a string if any
    """
    return db_tools.update_with_script_no_exception(db_file, CLEAR_TABLES_SQL)
