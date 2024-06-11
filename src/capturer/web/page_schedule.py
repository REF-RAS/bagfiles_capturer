# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import base64, traceback
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from enum import Enum
import dash
from dash import html, dcc, callback, Input, Output, State, dash_table
import dash_bootstrap_components as dbc
# project modules
from dash.exceptions import PreventUpdate
from .upload_helper import process_uploaded_excel_file
from tools import type_tools
from tools.logging_tools import logger
from capturer.model import ASSET_FILE_MANAGER, DAO, CapturerDAO

dash.register_page(__name__)

# -- define the GUI components of this page
class SetupPage():
    def __init__(self, app):
        self.app = app 
        self.worksheet_names = ['Schedule']
        self._define_page()
    
    def layout(self, validate=False):
        return self._layout

    def _define_page(self):
         
        schedule_import_confirm_dialog = dcc.ConfirmDialog(id='schedule_import_confirm_dialog',
            message='All schedule data will be cleared! Are you sure you want to continue?',)
        
        message_alert = dbc.Alert('The schedule is cleared', id='schedule_import_message_alert', dismissable=True,
                                 is_open=False, className='mx-auto mt-4 col-4')
        
        upload_area = dcc.Upload(id='schedule_import_upload_button', children=html.Div([
            'Drag and Drop or ', html.A('Select an Excel or CSV file')]), style={
            'width': '100%', 'height': '60px', 'lineHeight': '60px',
            'borderWidth': '1px', 'borderStyle': 'dashed', 'borderRadius': '5px',
            'textAlign': 'center', 'margin': '10px'}, multiple=False)
        
        self.upload_panel = dbc.Row([
            dbc.Col([
                html.H4(dbc.Badge('UPDATE SCHEDULE', className='ms-1 me-2', color='white', text_color='secondary')),
                html.P('Select the worksheets of the excel file to be imported.', style={'display': 'inline-block'}),
                dcc.Checklist(self.worksheet_names, self.worksheet_names, id='worksheet_checklist', 
                                                     inline=True, labelClassName='ms-3 me-3', inputClassName='m-1', style={'display': 'inline-block'}),
                upload_area,
                html.P('Warning! The current schedule will be deleted and replaced by the new schedule', className='text-danger'),
            ], className='col-12 text-center border')
        ], className='mx-auto col-10')

        # -- datatable for displaying schedule
        self._datatable = dash_table.DataTable(id='schedule_list_table')

        self._database_panel = html.Div([
            html.H4(dbc.Badge('PENDING CAPTURE SCHEDULE', className='ms-1 me-2', color='white', text_color='secondary')),
            dbc.Row([self._datatable], className='mx-auto col-12'),
            ], className='col-10 mx-auto text-center border mt-2')

        # -- putting the GUI components together 
        rows = html.Div(id='scan-body',children = [
            dcc.Store(id='schedule_import_uploaded_content'),
            dcc.Store(id='refresh_schedule_list_table_store'),
            dbc.Row(html.H4(id = 'title', children = 'Bag Files Capture Schedule', className='mt-3 mb-3')),
            self.upload_panel,
            self._database_panel,
            schedule_import_confirm_dialog,
            message_alert
        ])
        self._layout = dbc.Container(rows, fluid=True)

        self.app.callback([Output('schedule_import_message_alert', 'is_open'),
                           Output('schedule_import_message_alert', 'children'),
                           Output('refresh_schedule_list_table_store', 'data')],
            [Input('schedule_import_confirm_dialog', 'submit_n_clicks'),
             State('schedule_import_uploaded_content', 'data')], 
            prevent_initial_call=True)(self._confirm_import_button_pressed())
     
        self.app.callback([Output('schedule_import_confirm_dialog', 'displayed'),
                           Output('schedule_import_uploaded_content', 'data')],
            [Input('schedule_import_upload_button', 'contents'),
                State('schedule_import_upload_button', 'filename'),
                State('schedule_import_upload_button', 'last_modified'),
                State('worksheet_checklist', 'value')
            ], 
            prevent_initial_call=True)(self._import_selected_file())

        self.app.callback(Output('schedule_list_table', 'data'),
            [Input('refresh_schedule_list_table_store', 'data')], prevent_initial_call=False)(self._update_table())      
    
    # - the dataframe containing the schedule
    def _define_model(self):
        try:
            model = DAO.list_schedule_pending()
            model = model.drop(['the_date', 'the_time'], axis=1)
            model['timestamp'] = model['timestamp'].apply(type_tools.timestamp_to_datestr)
            model['status'] = model['status'].apply(lambda x: CapturerDAO.ScheduleStates(x).name)
            model = model[['timestamp', 'duration', 'folder', 'prefix', 'status', ]]
            model.columns = ['Datetime', 'Duration(s)', 'Folder', 'Prefix', 'Status']
            self._model = model
        except:
            logger.error(f'Error: {traceback.format_exc()}')
            self._model = pd.DataFrame()
    
    # -- the callback for the confirm button
    def _confirm_import_button_pressed(self): 
        def confirm_import_button_pressed(submit_n_clicks, uploaded):
            if submit_n_clicks:
                error_list = process_uploaded_excel_file(uploaded['content'], uploaded['worksheets'], uploaded['filename'], uploaded['last_modified'])
                message = 'update schedule is successful'
                if len(error_list) > 0:
                    message = error_list
                return (True, message, 0) 
        return confirm_import_button_pressed  
     
    # -- the callback for the import file
    def _import_selected_file(self): 
        def import_selected_file(contents, filename, last_modified, worksheet_list):           
            uploaded = {'content': contents, 'filename': filename, 'last_modified': last_modified, 'worksheets': worksheet_list}
            return (True, uploaded)  
        return import_selected_file 

    # -- the callback for table update
    def _update_table(self):
        def update_table(data):
            self._define_model()
            if self._model is None:
                raise PreventUpdate
            return self._model.to_dict('records')
        return update_table  

    
