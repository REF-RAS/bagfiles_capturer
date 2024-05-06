# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import base64, traceback, numbers, shutil
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from enum import Enum
import dash
from dash import html, dcc, callback, Input, Output, State, dash_table
import dash_bootstrap_components as dbc
# project modules
from dash.exceptions import PreventUpdate
from tools import type_tools
from tools.logging_tools import logger
from capturer.model import ASSET_FILE_MANAGER, DAO, CONFIG
import capturer.model as model
import rospy, rostopic

dash.register_page(__name__)

# -- define the GUI components of this page
class ConsolePage():
    def __init__(self, app, refresh_cycle=5):
        self.app = app 
        self.refresh_cycle = refresh_cycle
        if self.refresh_cycle is None or type(self.refresh_cycle) not in (float, int):
            self.refresh_cycle = 5
        self.worksheet_names = ['Schedule']
        self._define_page()
    
    def layout(self, validate=False):
        return self._layout

    def _define_page(self):
        
        message_alert = dbc.Alert('The schedule is cleared', id='console_message_alert', dismissable=True,
                                 is_open=False, className='mx-auto mt-4 col-4')

        self._system_status_panel = html.Div([
            html.H4(dbc.Badge('SYSTEM STATUS', className='ms-1 me-2', color='white', text_color='secondary')),
            dbc.Row(dbc.Badge(id='console_system_status_state', className='fs-5', color='white', text_color='primary'), className='mx-auto col-8'),
            dbc.Row(html.P(id='console_system_status_message', style={'text-align':'left', 'font-family':'monospace'}), className='mx-auto col-8'),
            ], className='col-10 mx-auto text-center border mt-2', style={'height': 180,})        

        # -- datatable for displaying rostopics status
        self._rostopics_datatable = dash_table.DataTable(id='console_rostopic_status_table')

        self._rostopics_database_panel = html.Div([
            html.H4(dbc.Badge('ROSTOPIC STATUS', className='ms-1 me-2', color='white', text_color='secondary')),
            dbc.Row([self._rostopics_datatable], className='mx-auto col-12'),
            ], className='col-8 mx-auto text-center border mt-3')

        self._diskspace_datatable = dash_table.DataTable(id='console_diskspace_table')

        self._diskspace_panel = html.Div([
            html.H4(dbc.Badge('DISK SPACE', className='ms-1 me-2', color='white', text_color='secondary')),
            dbc.Row([self._diskspace_datatable], className='mx-auto col-12'),
            ], className='col-4 mx-auto text-center border mt-3')

        # -- datatable for displaying rostopics status 
        self._bagfiles_datatable = dash_table.DataTable(id='console_latest_bagfiles_table')

        self._bagfiles_database_panel = html.Div([
            html.H4(dbc.Badge('LATEST BAGFILES', className='ms-1 me-2', color='white', text_color='secondary')),
            dbc.Row([self._bagfiles_datatable], className='mx-auto col-12'),
            ], className='col-10 mx-auto text-center border mt-3')     

        # -- putting the GUI components together 
        rows = html.Div(id='scan-body',children = [
            dcc.Store(id='console_update_store'),
            dbc.Row(html.H4(id = 'title', children = 'System Status Console', className='mt-3 mb-3')),
            self._system_status_panel,
            dbc.Row([self._rostopics_database_panel, self._diskspace_panel,], className='col-10 mx-auto text-center mt-3'),
            self._bagfiles_database_panel,
            message_alert
        ])
        self._layout = dbc.Container(rows, fluid=True)

        self.app.callback(Output('console_rostopic_status_table', 'data'),
            [Input('console_update_store', 'data')], prevent_initial_call=False)(self._update_rostopic_status_table())   
     
        self.app.callback(Output('console_diskspace_table', 'data'),
            [Input('console_update_store', 'data')], prevent_initial_call=False)(self._update_diskspace_table())  
           
        self.app.callback(Output('console_latest_bagfiles_table', 'data'),
            [Input('console_update_store', 'data')], prevent_initial_call=False)(self._update_latest_bagfiles_table())  
                   

        self.app.callback([Output('console_system_status_state', 'children'),
                           Output('console_system_status_message', 'children'),],
            [Input('console_update_store', 'data')], prevent_initial_call=True)(self._update_system_status_panel())  

        self.app.callback([Output('console_update_store', 'data', allow_duplicate=True)],
            [Input('dashapp_interval_store', 'data')], prevent_initial_call=True)(self._update_all())   
            
    def _find_topic_class(self, row):
        topic_class_name = 'Unknown'
        try:
            topic_class:rospy.Message
            topic_class, topic_str, _ = rostopic.get_topic_class(row['topic_name'])
            topic_class_name = topic_class.__qualname__
            rospy.wait_for_message(row["topic_name"], topic_class, timeout=0.3) 
            return str(topic_class_name), '✅'
        except Exception as e:
            return str(topic_class_name), '❌'

    def _define_rostopics_model(self):
        try:
            model = DAO.list_rostopic_names()
            model = pd.DataFrame({'topic_name': model})
            model[['topic_class', 'status']] = model.apply(self._find_topic_class, axis=1, result_type = "expand")
            return model
        except:
            logger.error(f'Error: {traceback.format_exc()}')
            return pd.DataFrame()

    def _define_bagfiles_model(self):
        try:
            model = DAO.list_latest_bagfiles(10)
            model = model.drop(['folder', 'name'], axis=1)
            model['timestamp'] = model['timestamp'].apply(type_tools.timestamp_to_datestr)
            model.columns = ['ID', 'Datetime', 'Duration(s)', 'File Path', 'File Size (bytes)']
            return model
        except:
            return pd.DataFrame()
            
    def _define_diskspace_model(self):
        total, used, free = shutil.disk_usage(CONFIG.get('capturer.filestore'))
        model = pd.DataFrame(columns=('Parameters', 'Values'))
        model.loc[1] = ['Total', f'{total // (2**30)} GB']
        model.loc[2] = ['Used', f'{used // (2**30)} GB']
        model.loc[3] = ['Free', f'{free // (2**30)} GB']        
        return model

    # -- the callback for table update
    def _update_rostopic_status_table(self):
        def update_table(data):
            model = self._define_rostopics_model()
            if model is None:
                raise PreventUpdate
            return model.to_dict('records')
        return update_table  
    
    # -- the callback for table update
    def _update_system_status_panel(self):
        def update_system_status_panel(data):
            state:model.SystemStates = model.STATE.get()
            message = ''
            if state == model.SystemStates.CAPTURING:
                schedule = model.STATE.get_var('capture_info')
                message = schedule['exec_script']
            return (state.name, message,)
        return update_system_status_panel   
    
        # -- the callback for table diskspace
    def _update_diskspace_table(self):
        def update_diskspace_table(data):
            model = self._define_diskspace_model()
            if model is None:
                raise PreventUpdate
            return model.to_dict('records')
        return update_diskspace_table  
    
    # -- the callback for table bagfiles
    def _update_latest_bagfiles_table(self):
        def update_latest_bagfiles_table(data):
            model = self._define_bagfiles_model()
            if model is None:
                raise PreventUpdate
            return model.to_dict('records')
        return update_latest_bagfiles_table  

    # -- the callback for all update
    def _update_all(self):
        def update_all(n):
            if (n-1) % self.refresh_cycle != 0:
                raise PreventUpdate
            return (1,)
        return update_all  
