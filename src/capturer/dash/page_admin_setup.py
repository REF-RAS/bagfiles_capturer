# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import base64
import plotly.express as px
import plotly.graph_objects as go
from enum import Enum
import dash
from dash import html, dcc, callback, Input, Output, State, ctx
import dash_bootstrap_components as dbc
# project modules
from dash.exceptions import PreventUpdate
import capturer.database_setup as dbsetup
from tools.logging_tools import logger
from capturer.model import ASSET_FILE_MANAGER, DAO, CONFIG
from .upload_helper import process_uploaded_excel_file
from .bagfiles_helper import clear_bagfiles

dash.register_page(__name__)

# -- define the GUI components of this page
class AdminSetupPage():
    def __init__(self, app):
        self.app = app 
        self.worksheet_names = ['Account']
        self._define_page()
    
    def layout(self, validate=False):
        return self._layout

    def _define_page(self):
        reset_table_confirm_dialog = dcc.ConfirmDialog(id='admin_resetdb_confirm_dialog',
            message='All data in the database will be cleared! Are you sure you want to continue?',)    

        clear_bagfiles_confirm_dialog = dcc.ConfirmDialog(id='admin_clear_bagfiles_confirm_dialog',
            message='Bagfiles will be cleared! Are you sure you want to continue?',)    
        
        account_import_confirm_dialog = dcc.ConfirmDialog(id='admin_account_import_confirm_dialog',
            message='All existing accounts will be cleared! Are you sure you want to continue?',)
        
        message_alert = dbc.Alert('The schedule is cleared', id='admin_message_alert', dismissable=True,
                                 is_open=False, className='mx-auto mt-4 col-4')

        self.panel_reset_db = dbc.Col([
            html.H4(dbc.Badge('RESET DB TABLES', className='ms-1 me-2', color='white', text_color='secondary')),
             html.P('Press the button to reset the database', className='mt-3'),
            dbc.Button('Drop and Create Tables', id='import_table_reset_button', color='danger', className='w-50'), 
            html.P('Warning! All the DB tables will be deleted and then created', className='mt-3 text-danger'),
            html.P('(You will be required to confirm thrice)', className='mt-3 text-danger'),
            ], className='col-6 text-center border')
        
        self.panel_remove_bagfiles = dbc.Col([
            html.H4(dbc.Badge('CLEAR BAGFILES', className='ms-1 me-2', color='white', text_color='secondary')),
            html.P('Press one of the buttons for options of clearing bagfiles', className='mt-3'),
            dbc.Button('Clear Bagfiles', id='admin_clear_bagfile_button', color='warning', className='col-3 mt-2'), 
            html.Span(' older than '),
            dcc.Input(id='admin_clear_days_input', type='number', value='30', className='col-1'),
            html.Span(' days'),
            html.Br(),
            # dbc.Button('Clear Bagfiles (Older than 7 Days)', id='admin_clear_7days_button', color='warning', className='w-50 mt-2'), html.Br(),
            # dbc.Button('Clear Bagfiles (Older than 1 Month)', id='admin_clear_1month_button', color='warning', className='w-50 mt-2'), html.Br(),
            html.P('(You will be required to confirm twice)', className='mt-3 text-danger'),
            ], className='col-6 text-center border')
        
        self.panel_update_rostopics = dbc.Col([
            html.H4(dbc.Badge('UPDATE ROSTOPICS', className='ms-1 me-2', color='white', text_color='secondary')),
            html.P('The rostopics to be included in bag files. The defaults are loaded if left empty.', className='mt-3'),
            dcc.Textarea(id='admin_setup_rostopic_textarea', value='', style={'width': '90%', 'height': 180, 'font-family':'monospace'},),
            dbc.Button('Update', id='admin_setup_rostopic_button', color='primary', className='w-50'), 
            ], className='col-6 text-center border')        

        self.account_upload_area = dcc.Upload(id='admin_account_import_upload_button', children=html.Div([
            'Drag and Drop or ', html.A('Select an Excel or CSV file')]), style={
            'width': '100%', 'height': '60px', 'lineHeight': '60px',
            'borderWidth': '1px', 'borderStyle': 'dashed', 'borderRadius': '5px',
            'textAlign': 'center', 'margin': '10px'}, multiple=False)
        
        self.account_upload_panel = dbc.Col([
                html.H4(dbc.Badge('UPDATE ACCOUNTS', className='ms-1 me-2', color='white', text_color='secondary')),
                html.P('Select the worksheets of the excel file to be imported.', style={'display': 'inline-block'}),
                dcc.Checklist(self.worksheet_names, self.worksheet_names, id='admin_account_worksheet_checklist', 
                                                     inline=True, labelClassName='ms-3 me-3', inputClassName='m-1', style={'display': 'inline-block'}),
                self.account_upload_area,
                html.P('Warning! The current accounts will be deleted and replaced by the new ones', className='text-danger'),
            ], className='col-6 text-center border')


        # -- putting the GUI components together 
        rows = html.Div(id='scan-body',children = [
            dcc.Store(id='account_import_uploaded_content'),
            dcc.Store(id='clear_bagfiles_type_store'),
            dbc.Row(html.H4(id = 'title', children = 'System Setup', className='mt-3 mb-3')),
            dbc.Row([self.panel_reset_db, self.panel_remove_bagfiles], className='mx-auto col-10'),
            dbc.Row([self.panel_update_rostopics, self.account_upload_panel], className='mx-auto col-10'),
            reset_table_confirm_dialog,
            clear_bagfiles_confirm_dialog,
            account_import_confirm_dialog,
            message_alert
        ])
        self._layout = dbc.Container(rows, fluid=True)
        
        self.app.callback([Output('admin_resetdb_confirm_dialog', 'displayed'),
                           Output('admin_resetdb_confirm_dialog', 'submit_n_clicks')],
            [Input('import_table_reset_button', 'n_clicks')], 
            prevent_initial_call=True)(self._table_reset_button_pressed())
        
        self.app.callback([ Output('admin_resetdb_confirm_dialog', 'displayed', allow_duplicate=True),
                            Output('admin_message_alert', 'is_open', allow_duplicate=True),
                           Output('admin_message_alert', 'children', allow_duplicate=True),],
            [Input('admin_resetdb_confirm_dialog', 'submit_n_clicks')], 
            prevent_initial_call=True)(self._confirm_reset_button_pressed())
        
        self.app.callback([Output('admin_setup_rostopic_textarea', 'value'),
                           Output('admin_message_alert', 'is_open', allow_duplicate=True),
                           Output('admin_message_alert', 'children', allow_duplicate=True),],
            [Input('admin_setup_rostopic_button', 'n_clicks'),
             State('admin_setup_rostopic_textarea', 'value'),
                 ], prevent_initial_call='initial_duplicate')(self._rostopic_button_pressed())
                
        self.app.callback([Output('admin_clear_bagfiles_confirm_dialog', 'displayed'),
                           Output('admin_clear_bagfiles_confirm_dialog', 'submit_n_clicks'),
                           Output('clear_bagfiles_type_store', 'data')],
            [Input('admin_clear_bagfile_button', 'n_clicks'),], prevent_initial_call=True)(self._clear_bagfile_button_pressed())
 
        self.app.callback([Output('admin_clear_bagfiles_confirm_dialog', 'displayed', allow_duplicate=True),
                            Output('admin_message_alert', 'is_open', allow_duplicate=True),
                           Output('admin_message_alert', 'children', allow_duplicate=True),],
            [Input('admin_clear_bagfiles_confirm_dialog', 'submit_n_clicks'),
            Input('clear_bagfiles_type_store', 'data'),
            State('admin_clear_days_input', 'value')], 
            prevent_initial_call=True)(self._confirm_clear_bagfile_button_pressed())
        
        self.app.callback([Output('admin_message_alert', 'is_open'),
                           Output('admin_message_alert', 'children')],
            [Input('admin_account_import_confirm_dialog', 'submit_n_clicks'),
             State('account_import_uploaded_content', 'data')], 
            prevent_initial_call=True)(self._confirm_account_import())
     
        self.app.callback([Output('admin_account_import_confirm_dialog', 'displayed'),
                           Output('account_import_uploaded_content', 'data')],
            [Input('admin_account_import_upload_button', 'contents'),
                State('admin_account_import_upload_button', 'filename'),
                State('admin_account_import_upload_button', 'last_modified'),
                State('admin_account_worksheet_checklist', 'value')
            ], 
            prevent_initial_call=True)(self._account_import_selected_file())
        
    def _table_reset_button_pressed(self):
        def table_reset_button_pressed(reset_button):
            return (True, 0,)
        return table_reset_button_pressed
    
    def _confirm_reset_button_pressed(self):
        def confirm_reset_button_pressed(submit_n_clicks):
            if submit_n_clicks <= 2:
                return (True, False, '')
            else:
                error = dbsetup.drop_tables(DAO.db_file)
                if error is None:
                    error = dbsetup.create_tables(DAO.db_file)
                    if error is None:
                        return (False, True, f'Successfully reset the database tables to a blank state')
                    else:
                       return (False, True, f'Error in creating tables: {error}') 
                else:
                    return (False, True, f'Error in dropping tables: {error}')
        return confirm_reset_button_pressed
    
    def _rostopic_button_pressed(self):
        def rostopic_button_pressed(submit_n_clicks, text):
            if submit_n_clicks is not None:
                DAO.clear_rostopics()
                topic_names = text.split('\n')
                for topic_name in topic_names:
                    topic_name = topic_name.strip()
                    if len(topic_name) > 0:
                        DAO.add_rostopic(topic_name)
                return (text, True, f'Successfully updated rostopics')
            else:
                rostopic_names = DAO.list_rostopic_names()
                if rostopic_names is None:
                    return ('', False, '')
                return ('\n'.join(rostopic_names), False, '')
        return rostopic_button_pressed
    
    def _clear_bagfile_button_pressed(self):
        def clear_bagfile_button_pressed(clear):
            button_id = ctx.triggered_id if not None else 'No clicks yet'
            if button_id == 'admin_clear_bagfile_button':
                return (True, 0, 'clear')
            raise PreventUpdate
        return clear_bagfile_button_pressed

    def _confirm_clear_bagfile_button_pressed(self):
        def confirm_clear_bagfile_button_pressed(submit_n_clicks, type, days):
            if submit_n_clicks <= 1:
                return (True, False, '')
            else:
                if type == 'clear':
                    try:
                        days = int(days)
                    except Exception as e:
                        return (False, True, f'Invalid day value: {days}')
                    older_than = f'-{days} days'
                    clear_bagfiles(older_than)
                return (False, True, f'Successfully clear bagfiles older than {days} days')
        return confirm_clear_bagfile_button_pressed
    
    # -- the callback for import accounts
    def _confirm_account_import(self): 
        def confirm_account_import(submit_n_clicks, uploaded):
            if submit_n_clicks:
                error_list = process_uploaded_excel_file(uploaded['content'], uploaded['worksheets'], uploaded['filename'], uploaded['last_modified'])
                message = 'update account is successful'
                if len(error_list) > 0:
                    message = error_list
                return (True, message) 
        return confirm_account_import  
     
    # -- the callback for import accounts
    def _account_import_selected_file(self): 
        def account_import_selected_file(contents, filename, last_modified, worksheet_list):           
            uploaded = {'content': contents, 'filename': filename, 'last_modified': last_modified, 'worksheets': worksheet_list}
            return (True, uploaded)  
        return account_import_selected_file 