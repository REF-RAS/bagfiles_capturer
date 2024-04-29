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
from dash import html, dcc, callback, Input, Output, dash_table
import dash_bootstrap_components as dbc
from dash.exceptions import PreventUpdate 
# project modules
import capturer.model as model

dash.register_page(__name__)

class ConsolePage():
    def __init__(self, app):
        self.page_content = 'The console page'
        self._define_page()

    def layout(self):
        return self._layout

    def _define_page(self):
        self.page_content = dcc.Markdown()
        if self.page_content:
            self.page_content.children = self.page_content

        rows = html.Div(id='main_div', children = [
            dbc.Row(self.page_content, className='mx-auto col-9 border mt-3 mb-3'),
            ], )

        self._layout = dbc.Container(rows, fluid=True)  
