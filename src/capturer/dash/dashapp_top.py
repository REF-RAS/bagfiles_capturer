# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

# import libraries
import sys, os, signal, time, threading, json, webbrowser, openpyxl
import dash
import dash_bootstrap_components as dbc
from dash import html, dcc
from dash.dependencies import Input, Output, State
from flask import Flask
# ros modules
import rospy, message_filters
# project modules
from tools.yaml_tools import YamlConfig
import capturer.model as model

from . import themes

server = Flask(__name__)
APP = dash.Dash(__name__, 
                external_stylesheets=[dbc.themes.BOOTSTRAP],  
                server=server,
                meta_tags=[{"name": "viewport", "content": "width=device-width"}],
                suppress_callback_exceptions=True)

APP.config['suppress_callback_exceptions'] = True

# -- load the pages for the dash application
from . import page_console, page_admin_setup

# -- Create a dash application object
class DashAppTop():
    def __init__(self, capturer_config):
        global APP
        self.capturer_config = capturer_config
        self.DASH_HOST = capturer_config.get('capturer.dash.host')
        self.DASH_PORT = capturer_config.get('capturer.dash.port')
        self.SYSTEM_TIMER = capturer_config.get('capturer.system.timer', 1) * 1000

        self.app = APP
        self.flask_server = self.app.server
        # - define the pages
        self._console_page = page_console.ConsolePage(self.app)
        self._setup_page = page_admin_setup.AdminSetupPage(self.app)

        # - initialize the application
        self._define_app()

    def _define_app(self):
        self._navbar_with_menu = dbc.NavbarSimple(
                    children=[
                        dbc.NavItem(dbc.NavLink('Console', href='/page_console')),
                        dbc.NavItem(dbc.NavLink('Schedule', href='/page_setup')),
                    ],
                    brand=html.H3('SHORTS Bagfile Capturer Coordinator'), 
                    brand_href='/page_console', color='#cce6ff', className='fs-4 text')

        self._navbar_simple = dbc.NavbarSimple(
                    brand=html.H3('Bagfile Capturer Coordinator'), color='#99cccc', className='fs-4 text')      

        self._nav_placeholder = html.Div(id='nav_placeholder')
        
        self.app.layout = html.Div([ 
            # for the dash system timer
            dcc.Store(id='dashapp_operator_interval_store'),
            dcc.Interval(id='system_interval', interval=self.SYSTEM_TIMER, n_intervals=0),
            dcc.Location(id='url', refresh=False),
            self._nav_placeholder, 
            html.Div(id='page_content', children=[]), 
        ])
        # ----- the dash callbacks
        self.app.callback([Output('page_content', 'children'),
                           Output('nav_placeholder', 'children')],
              [Input('url', 'pathname')])(self._display_page())
        
        self.app.callback([Output('dashapp_operator_interval_store', 'data')],
              [Input('system_interval', 'n_intervals')],
              [State('url', 'pathname')])(self._dash_system_timer())
    
    def start(self):
        rospy.loginfo('operation agent: starting the dash flask server')
        self.app.run_server(host=self.DASH_HOST, port=self.DASH_PORT, debug=self.capturer_config.get('capturer.dash.debug.mode', True))

    def stop(self, *args, **kwargs):
        print('operation agent: the dash flask server is being shutdown')
        time.sleep(2)
        sys.exit(0)

    # -- flask dash callback for url handling
    def _display_page(self):
        def display_page(pathname):
            page_content = ''
            nav_content = self._navbar_with_menu
            if pathname == '/page_console':
                page_content = self._console_page.layout(validate=True)
            elif pathname == '/page_setup':
                page_content = self._setup_page.layout()        
            elif pathname == '/': 
                page_content = self._console_page.layout(validate=True)
            else: # if redirected to unknown link
                page_content = '404 Page Error! Please choose a link'
            return (page_content, nav_content,)
        return display_page
    
    # -- dash callback for system interval timer
    def _dash_system_timer(self):
        def dash_system_timer(n, pathname):
            model.CALLBACK_MANAGER.fire_event(model.CallbackTypes.TIMER)
            return (1,)
        return dash_system_timer
    
    @server.route("/<path>")
    def update_title(path):
        APP.title = 'SHORTS CAPTURER'
        return APP.index()
        
    