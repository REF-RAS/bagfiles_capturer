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


# ----- the common modules
import os, json, random, copy, glob, collections, sys, logging, yaml
import tools.db_tools as db_tools
import tools.file_tools as file_tools

CREATE_TABLES_SQL = """ 
CREATE TABLE IF NOT EXISTS general_config (
    name text PRIMARY KEY,
    value text
);

"""

DROP_TABLES_SQL = """
DROP TABLE IF EXISTS scan_pattern;
"""

CLEAR_SCAN_TABLES_SQL = """
DELETE FROM scan;

"""

def drop_tables(db_file):
    os.makedirs(file_tools.get_parent(db_file), exist_ok=True)
    return db_tools.update_with_script_no_exception(db_file, DROP_TABLES_SQL)

def create_tables(db_file):
    os.makedirs(file_tools.get_parent(db_file), exist_ok=True)
    return db_tools.update_with_script_no_exception(db_file, CREATE_TABLES_SQL)

def clear_all_scan_tables(db_file):
    return db_tools.update_with_script_no_exception(db_file, CLEAR_SCAN_TABLES_SQL)
