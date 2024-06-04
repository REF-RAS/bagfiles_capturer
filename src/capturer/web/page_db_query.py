# Copyright 2023 - Andrew Kwok Fai LUI, Centre for Robotics
# and the Queensland University of Technology
#
__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2023, The CGRAS Project'
__license__ = 'GPL'
__version__ = '0.0.1'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import plotly.express as px
import plotly.graph_objects as go
from enum import Enum
import dash
from dash import html, dcc, callback, Input, Output, State, dash_table, ctx
import dash_bootstrap_components as dbc
from dash.exceptions import PreventUpdate 
# project modules
from tools import db_tools, type_tools
from capturer.model import DAO
import capturer.model as model
import pandas as pd

dash.register_page(__name__)

class DBQueryPage():
    def __init__(self, app):
        self.app = app
        self.sql_list = []
        self._error_placeholder = None
        self._define_page()

    def get_tablenames(self):
        return db_tools.list_table_names(DAO.db_file)    

    def layout(self):
        return self._layout

    def _define_page(self):
        self._error_placeholder = html.Div(id='error_div', className='col-12')
        self._dropdown_placeholder = html.Div(className='col-12')
        self._sql_dropdown = dcc.Dropdown(self.sql_list, id='sql_list_dropdown', searchable=False)
        self._dropdown_placeholder.children = self._sql_dropdown
        self._datatable = dash_table.DataTable(id='db_query_table', page_current=0, page_size=100, page_action='custom')
        rows = html.Div(id='main_div', children = [
            dbc.Row(html.H4(id = 'title', children = 'Query DB Tables', className='mt-3 mb-3')),
            dbc.Row(self._dropdown_placeholder, className='mt-3 mb-3'),
            dbc.Row([dbc.Col(dcc.Input(id='db_query_textbox_input', type='text', placeholder='SQL', className='col-12 d-inline-block'), 
                             className='col-10'),
                    dbc.Col(dbc.Button('Query', id='db_query_button', n_clicks=0)),
                    dbc.Col(dbc.Button('Update', id='db_update_button', n_clicks=0))]),
            dbc.Row(self._error_placeholder),
            dbc.Row([html.Div(html.H6(dbc.Badge('SQL RESULTS', color='white', text_color='primary'))),
                html.Div(self._datatable)
            ], id='db_query_table_div', style={'display': 'none'})
        ])
        self._layout = dbc.Container(rows, fluid=True) 
        # - define callback
        self.app.callback([Output('db_query_table', 'data'),
                           Output('sql_list_dropdown', 'options'),
                           Output('error_div', 'children'),
                           Output('db_query_table_div', 'style')],
                            [
                            Input('db_query_button', 'n_clicks'),
                            Input('db_update_button', 'n_clicks'),
                            State('db_query_textbox_input', 'value'),
                            Input('db_query_table', 'page_current'),
                            Input('db_query_table', 'page_size')
                            ], prevent_initial_call=True)(self._update_table())
        
        self.app.callback(Output('db_query_textbox_input', 'value'),
                          Input('sql_list_dropdown', 'value'), prevent_initial_call=True)(self._update_sql_textbox())
    
    def _update_table(self):
        def update_table(query_button, update_button, sql, page_current, page_size):
            self._error_placeholder.children = None
            if not sql or len(sql.strip()) == 0:
                raise PreventUpdate
            try:
                button_id = ctx.triggered_id if not None else 'No clicks yet'
                if button_id == 'db_query_button':
                    df = db_tools.query_paged(DAO.db_file, sql, page_size, page_current * page_size)
                    if 'start_time' in df.columns:
                        df['start_time'] = df['start_time'].apply(type_tools.timestamp_to_datestr)
                    if 'commit_time' in df.columns:
                        df['commit_time'] = df['commit_time'].apply(type_tools.timestamp_to_datestr)
                    if 'timestamp' in df.columns:
                        df['timestamp'] = df['timestamp'].apply(type_tools.timestamp_to_datestr)
                elif button_id == 'db_update_button':
                    rowcount = db_tools.update(DAO.db_file, sql)
                    df = pd.DataFrame(['Results', rowcount])

                if sql not in self.sql_list:
                    self.sql_list.append({'label': sql, 'value': sql})
                    self.sql_list = self.sql_list[-10:]
                    self._sql_dropdown.__setattr__('options', self.sql_list)
                return (df.to_dict('records'), self.sql_list, html.P(), {'display': 'block'},)
            except Exception as e:
                error_alert = dbc.Alert(f'Error: {e}', dismissable=True, is_open=True, color='danger')
                return (pd.DataFrame().to_dict('records'), self.sql_list, error_alert, {'display': 'none'}, )
        return update_table

    def _update_sql_textbox(self):
        def update_sql_textbox(value):
            return value
        return update_sql_textbox