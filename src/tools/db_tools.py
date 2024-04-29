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
import sqlite3
import pandas as pd

# --- returns a db connection object to the db_file
def create_connection(db_file):
    try:
        conn = sqlite3.connect(db_file)
        return conn
    except sqlite3.Error as e:
        raise e
    
# --- clear the content of a table in the specified DB
def clear_table(db_file, tablename):
    with create_connection(db_file) as conn: 
        c = conn.cursor()          
        c.execute(f'DELETE FROM {tablename}' )
        conn.commit()
        return c.rowcount

# return a list of names of the tables
def list_table_names(db_file):
    with create_connection(db_file) as conn: 
        c = conn.cursor()   
        results = c.execute('SELECT name FROM sqlite_master WHERE type=\'table\'')
        return [item[0] for item in results]

# --- dump the content of a table as a dataframe
def dump_table_df(db_file, tablename, limit=None, offset=0):
    with create_connection(db_file) as conn: 
        if limit:
            return pd.read_sql(f'SELECT * FROM {tablename} LIMIT {limit} OFFSET {offset}', conn)
        else:
            return pd.read_sql(f'SELECT * FROM {tablename}', conn)

# --- execute sql query the results as a dataframe
def update(db_file, sql, *args):
    assert len(args) == 0 or (len(args) == 1 and type(args[0]) == tuple), 'sql statement parameters must be in a tuple'
    with create_connection(db_file) as conn:
        c = conn.cursor()
        c.execute(sql, *args)
        conn.commit()
        return c.rowcount

def update_with_script(db_file, sql_script, *args):
        assert len(args) == 0 or (len(args) == 1 and type(args[0]) == tuple), 'sql statement parameters must be in a tuple'
        with create_connection(db_file) as conn:
            c = conn.cursor()
            c.executescript(sql_script, *args)
            conn.commit()
            return c.rowcount

def update_with_script_no_exception(db_file, sql_script, *args):
    try:
        update_with_script(db_file, sql_script, *args)
        return None
    except Exception as e:
        return str(e)
        
# --- execute sql query the results as a dataframe
# formats can be None or one of the following
# 'dict' (default) : dict like {column -> {index -> value}}
# 'list' : dict like {column -> [values]}
# 'series' : dict like {column -> Series(values)}
# 'split' : dict like {'index' -> [index], 'columns' -> [columns], 'data' -> [values]}
# 'records' : list like [{column -> value}, … , {column -> value}]
# 'index' : dict like {index -> {column -> value}}
def query(db_file, sql, params=None, format=None):
    with create_connection(db_file) as conn:
        if params is None:
            df = pd.read_sql(sql, conn)
        else:
            assert type(params) == tuple, 'sql statement parameters must be in a tuple'
            df = pd.read_sql(sql, conn, params=params)
        if format is None:
            return df
        assert format in ['dict', 'list', 'series', 'split', 'records', 'index']
        return df.to_dict(orient=format)

def query_paged(db_file, sql, limit=None, offset=0, params=None, format=None):
    with create_connection(db_file) as conn:
        if limit:
            sql = f'{sql} LIMIT {limit} OFFSET {offset}'
        return query(db_file, sql, params=params, format=format)

# --- execute sql query that returns one object
def query_for_list(db_file, sql, *args):
    with create_connection(db_file) as conn:
        try:
            results = conn.execute(sql, *args).fetchall()
            result_list = [item[0] for item in results]
            return result_list
        except Exception as e:
            raise

# --- execute sql query that returns a dict of the first row
def query_for_dict(db_file, sql, *args):
    with create_connection(db_file) as conn:
        try:
            conn.row_factory = sqlite3.Row
            result = conn.execute(sql, *args).fetchmany(1)
            if len(result) == 0:
                return None
            result_list = {k: result[0][k] for k in result[0].keys()}
            return result_list
        except Exception as e:
            raise

# --- execute sql query that returns one object
def query_for_object(db_file, sql, *args):
    with create_connection(db_file) as conn:
        try:
            result = conn.execute(sql, *args).fetchone()
            if result is None or len(result) == 0:
                return None
            return result[0]
        except Exception as e:
            raise

# --- execute sql query that returns a list of dicts, each of which is a record (row) in the results
def query_as_list_of_dicts(db_file, sql, *args):
    with create_connection(db_file) as conn: 
        conn.row_factory = sqlite3.Row
        try:
            results = conn.execute(sql, *args).fetchall()
            result_list = [{k: item[k] for k in item.keys()} for item in results]
            return result_list
        except Exception as e:
            raise