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

# the global variables

#### -- MODEL: system state model
class SystemState():
    def __init__(self, init_state):
        self.state_lock = threading.Lock()
        self.station_name = None
        self.scan_id = None
        # error related information
        self.error_source = None
        self.error_message = None
        # list for storing info log messages
        self.log_info = []
        self.log_info_lock = threading.Lock()
        # list for modal alert messages
        self.alert_message = []
        self.alert_message_lock = threading.Lock()
        # check the status concerning if any scanning task is incomplete
        self.state = init_state
        self.previous_state = None
        self.last_update_time = time.time()
        self.changed_for_called = collections.defaultdict(lambda: False)
        # variables
        self.vars = dict()
        self.vars_lock = threading.Lock()

    def get_state(self, caller=None):
        self.state_lock.acquire()
        try:
            if caller is not None:
                self.changed_for_called[caller] = False
            return self.state
        finally:
            self.state_lock.release()

    def has_changed_state(self, caller):
        self.state_lock.acquire()
        try:        
            return self.changed_for_called.get(caller, False)
        finally:
            self.state_lock.release()

    def update_state(self, new_state, current_state=None):
        # -- current state must match if given
        self.state_lock.acquire()
        try:
            if current_state is None or self.state == current_state:
                if self.state != new_state:
                    for caller in self.changed_for_called.keys():
                        self.changed_for_called[caller] = True
                self.previous_state = self.state
                self.state = new_state
                self.last_update_time = time.time()
        finally:
            self.state_lock.release()
            
    def is_state(self, query_state):
        self.state_lock.acquire()
        try:
            return self.state == query_state
        finally:
            self.state_lock.release()
    
    def time_lapsed_since_update(self):
        if self.last_update_time is None:
            return None
        return time.time() - self.last_update_time
    
    def get_previous_state(self):
        return self.previous_state

    # -- alert messages
    def add_alert_message(self, message):
        self.alert_message_lock.acquire()
        try:
            self.alert_message.append(message)
        finally:
            self.alert_message_lock.release()
            
    def next_alert_message(self):
        self.alert_message_lock.acquire()
        try:
            if len(self.alert_message) == 0:
                return None
            return self.alert_message.pop(0)
        finally:
            self.alert_message_lock.release()            
    # -- info log
    def add_log_info(self, info, max_keep=10):
        self.log_info_lock.acquire()
        try:
            self.log_info.append(info)
            max_keep = min(max_keep, len(self.log_info))
            self.log_info = self.log_info[-max_keep:]
        finally:
            self.log_info_lock.release()
    def get_log_info(self):
        self.log_info_lock.acquire()
        try:
            return list(self.log_info)
        finally:
            self.log_info_lock.release()
    # -- error log
    def set_error(self, error_source, error_message):
        self.error_source = error_source
        self.error_message = error_message
        
    def clear_error(self):
        self.error_source = self.error_message = None
        
    def get_error_message(self):
        return self.error_message
    
    def get_error(self):
        return self.error_source, self.error_message   
    
    # -- passing variables between states
    def set_var(self, name, value):
        self.vars_lock.acquire()
        try:
            self.vars[name] = value
        finally:
            self.vars_lock.release()

    def get_var(self, name, default=None):
        self.vars_lock.acquire()
        try:
            if name in self.vars:
                return self.vars[name]
            return default
        finally:
            self.vars_lock.release()
            
    def del_var(self, name):
        self.vars_lock.acquire()
        try:
            if name in self.vars:
                del self.vars[name]      
        finally:
            self.vars_lock.release()
                
    def __str__(self):
        return f'state: {self.state} measurement_name: {self.station_name} scan_id: {self.scan_id}'

#### -- MODEL: callback manager
class CallbackManager():
    def __init__(self):
        self.callbacks = dict()

    def set_listener(self, event, listener):
        self.callbacks[event] = listener

    def fire_event(self, event, *args):
        if event in self.callbacks:
            self.callbacks[event](event, *args)

#### -- MODEL: state manager
class StateManager():
    def __init__(self, initial_state=None):   
        self.state_lock = threading.Lock()
        self.state = initial_state
        self.info = None
        self.last_update_time = None
        self.last_change_time = None
        # variables
        self.vars = dict()
        self.vars_lock = threading.Lock()
    def get(self):
        self.state_lock.acquire()
        try:
            return self.state
        finally:
            self.state_lock.release()
    def get_with_info(self):
        self.state_lock.acquire()
        try:
            return (self.state, self.info,)
        finally:
            self.state_lock.release()            
    def update(self, new_state, info=None):
        self.state_lock.acquire()
        try:
            if self.state != new_state:
                self.last_change_time = time.time()
            self.state = new_state
            self.info = info
            self.last_update_time = time.time()
        finally:
            self.state_lock.release()

    def is_state(self, query_state):
        self.state_lock.acquire()
        try:
            return self.state == query_state
        finally:
            self.state_lock.release()

    def time_lapsed_since_update(self):
        if self.last_update_time is None:
            return None
        return time.time() - self.last_update_time
    
    def time_lapsed_since_change(self):
        if self.last_change_time is None:
            return None
        return time.time() - self.last_change_time    
    
    # -- passing variables between states
    def set_var(self, name, value):
        self.vars_lock.acquire()
        try:
            self.vars[name] = value
        finally:
            self.vars_lock.release()

    def get_var(self, name, default=None):
        self.vars_lock.acquire()
        try:
            if name in self.vars:
                return self.vars[name]
            return default
        finally:
            self.vars_lock.release()
            
    def del_var(self, name):
        self.vars_lock.acquire()
        try:
            if name in self.vars:
                del self.vars[name]      
        finally:
            self.vars_lock.release()
    def __str__(self):
        return self.state
