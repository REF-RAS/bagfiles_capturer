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
from dash import html, dcc, callback, Input, Output, State
import dash_bootstrap_components as dbc
# project modules
from dash.exceptions import PreventUpdate
from .upload_spec_helper import process_uploaded_spec_file
import imaging.imaging_database_setup as dbsetup
from imaging.imaging_dao import DAO
from capturer.model import ASSET_FILE_MANAGER

dash.register_page(__name__)

# -- define the GUI components of this page
class AdminSetupPage():
    def __init__(self, app):
        self.app = app 
        self.worksheet_names = ['Tile', 'Tank', 'Station', 'ScanPattern']
        self._define_page()
    
    def layout(self):
        return self._layout

    def _define_page(self):
        reset_table_confirm_dialog = dcc.ConfirmDialog(id='reset_table_confirm_dialog',
            message='All data in the database will be cleared! Are you sure you want to continue?',)   
        import_clear_session_confirm_dialog = dcc.ConfirmDialog(id='import_clear_session_confirm_dialog',
            message='All captured images and the associated information will be cleared! Are you sure you want to continue?',)      
        import_confirm_dialog = dcc.ConfirmDialog(id='import_delete_confirm_dialog',
            message='The tank/tile related data in the database will be cleared! Are you sure you want to continue?',)
        message_alert = dbc.Alert('The specifications are cleared', id='import_message_alert', dismissable=True,
                                 is_open=False, className='mx-auto mt-4 col-4')
        process_uploaded_spec_file
        self._option_1 = dbc.Col([
            html.H4(dbc.Badge('RESET DB TABLES', className='ms-1 me-2', color='white', text_color='secondary')),
             html.P('Press the button to reset the database', className='mt-3'),
            dbc.Button('Drop and Create Tables', id='import_table_reset_button', color='danger', className='w-50'), 
            html.P('Warning! All the DB tables will be deleted and then created', className='mt-3 text-danger'),
            html.P('(You will be required to confirm thrice)', className='mt-3 text-danger'),
            ], className='col-6 text-center border')
        
        upload_area = dcc.Upload(id='import_upload_button', children=html.Div([
            'Drag and Drop or ', html.A('Select an Excel or CSV file')]), style={
            'width': '100%',
            'height': '60px',
            'lineHeight': '60px',
            'borderWidth': '1px',
            'borderStyle': 'dashed',
            'borderRadius': '5px',
            'textAlign': 'center',
            'margin': '10px'
        }, multiple=False)
        
        self._option_2 = dbc.Col([
            html.H4(dbc.Badge('IMPORT TILE/TANK SPECIFICATION', className='ms-1 me-2', color='white', text_color='secondary')),
            html.P('Select the worksheets of the excel file to be imported. Note: the TileConfig worksheet is always imported'),
            dcc.Checklist(self.worksheet_names, self.worksheet_names, id='worksheet_checklist', 
                                                     inline=True, labelClassName='ms-3 me-3', inputClassName='m-1'),
            upload_area,
            html.P('Warning! The current specifications will be deleted and replaced by the new specification', className='text-danger'),
            ], className='col-6 text-center border')

        self._option_3 = dbc.Col([
            html.H4(dbc.Badge('RESET DB TABLES', className='ms-1 me-2', color='white', text_color='secondary')),
             html.P('Press the button to remove the data from image capture sessions', className='mt-3'),
            dbc.Button('Clear All Image Capture Sessions', id='import_clear_session_button', color='danger', className='w-50'), 
            html.P('All the DB tables associated with image captures will be clear.', className='mt-3 text-danger'),
            ], className='col-6 text-center border')

        self._option_panel = html.Div([
            dbc.Row([html.H4('Available DB Functions for Admin (Use with Caution)')], className='mx-auto col-8 text-center'),
            dbc.Row([html.H4('These tools must not be used during an image capture session')], className='mx-auto col-8 text-center text-danger'),
            dbc.Row([self._option_1, self._option_2], className='mx-auto col-10'),
            dbc.Row([self._option_3,], className='mx-auto col-10'),
            ], className='col-12')

        # -- putting the GUI components together 
        rows = html.Div(id='scan-body',children = [
            dcc.Store(id='op_import_uploaded_content'),
            dbc.Row(html.H4(id = 'title', children = 'DB Setup', className='mt-3 mb-3')),
            self._option_panel,

            import_confirm_dialog,
            reset_table_confirm_dialog,
            import_clear_session_confirm_dialog,
            message_alert
        ])
        self._layout = dbc.Container(rows, fluid=True)

        self.app.callback([Output('import_message_alert', 'is_open'),
                           Output('import_message_alert', 'children'),],
            [Input('import_delete_confirm_dialog', 'submit_n_clicks'),
             State('op_import_uploaded_content', 'data')], 
            prevent_initial_call=True)(self._confirm_import_button_pressed())
     
        self.app.callback([Output('import_delete_confirm_dialog', 'displayed'),
                           Output('op_import_uploaded_content', 'data')],
            [Input('import_upload_button', 'contents'),
                State('import_upload_button', 'filename'),
                State('import_upload_button', 'last_modified'),
                State('worksheet_checklist', 'value')
            ], 
            prevent_initial_call=True)(self._import_selected_file())
        
        self.app.callback([Output('reset_table_confirm_dialog', 'displayed'),
                           Output('reset_table_confirm_dialog', 'submit_n_clicks')],
            [Input('import_table_reset_button', 'n_clicks')], 
            prevent_initial_call=True)(self._table_reset_button_pressed())
        
        self.app.callback([ Output('reset_table_confirm_dialog', 'displayed', allow_duplicate=True),
                            Output('import_message_alert', 'is_open', allow_duplicate=True),
                           Output('import_message_alert', 'children', allow_duplicate=True),],
            [Input('reset_table_confirm_dialog', 'submit_n_clicks')], 
            prevent_initial_call=True)(self._confirm_reset_button_pressed())
        
        self.app.callback([Output('import_clear_session_confirm_dialog', 'displayed'),
                           Output('import_clear_session_button', 'submit_n_clicks')],
            [Input('import_clear_session_button', 'n_clicks')], 
            prevent_initial_call=True)(self._import_clear_session_button_pressed())
        
        self.app.callback([ Output('import_clear_session_confirm_dialog', 'displayed', allow_duplicate=True),
                            Output('import_message_alert', 'is_open', allow_duplicate=True),
                           Output('import_message_alert', 'children', allow_duplicate=True),],
            [Input('import_clear_session_confirm_dialog', 'submit_n_clicks')], 
            prevent_initial_call=True)(self._confirm_clear_session_button_pressed())

    # -- the callback for the confirm button
    def _confirm_import_button_pressed(self): 
        def confirm_import_button_pressed(submit_n_clicks, uploaded):
            if submit_n_clicks:
                error_list = process_uploaded_spec_file(uploaded['content'], uploaded['filename'], uploaded['last_modified'], 
                                                        uploaded['worksheets'])
                message = 'import tile and tank specifications is successful'
                if len(error_list) > 0:
                    message = error_list
                return (True, message) 
        return confirm_import_button_pressed  
     
    # -- the callback for the import file
    def _import_selected_file(self): 
        def import_selected_file(contents, filename, last_modified, worksheet_list):           
            uploaded = {'content': contents, 'filename': filename, 'last_modified': last_modified, 'worksheets': worksheet_list}
            return (True, uploaded)  
        return import_selected_file 
    
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
    
    def _import_clear_session_button_pressed(self):
        def import_clear_session_button_pressed(reset_button):
            return (True, 0,)
        
        return import_clear_session_button_pressed
    
    def _confirm_clear_session_button_pressed(self):
        def confirm_clear_session_button_pressed(submit_n_clicks):
            error = DAO.clear_all_scan_tables()
            if not error: 
                try:   # ADDED 9-Nov-2023 untested
                    ASSET_FILE_MANAGER.remove_indices_folder()
                    ASSET_FILE_MANAGER.clear_sample_folder()
                    ASSET_FILE_MANAGER.generate_indices_folder() 
                except:
                    return (False, True, f'Error in clear image capture related files: {error}')
                return (False, True, f'Successfully clear the image capture related data')
            else:
                return (False, True, f'Error in clear image capture related data: {error}')

        return confirm_clear_session_button_pressed
    
