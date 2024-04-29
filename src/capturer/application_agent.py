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

# import libraries
import sys, os, signal, time, threading, json, webbrowser
# ros modules
import rospy, message_filters, actionlib
from std_msgs.msg import String, Header, Bool, Int8, Float32
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Image
# project modules
from tools.yaml_tools import YamlConfig
import capturer.model as capturer_model
from capturer.dash.dashapp_top import DashAppTop

class ApplicationCoordinator(object):
    def __init__(self, capturer_config):
        # save the config
        self.capturer_config = capturer_config
        # create lock for synchronization
        self.state_lock = threading.Lock()

        # - create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)
        rospy.on_shutdown(self.cb_shutdown)

        # define the state publisher
        # self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_state)

        # - register the callbacks
        # using the dash callback instead of the ros callback reduces the flickering on the web GUI
        capturer_model.CALLBACK_MANAGER.set_listener(capturer_model.CallbackTypes.TIMER, self._timer_callback)
           
        # robot acceptable state for operation
        # create the dash application   
        self.dash_app_operator = DashAppTop()
        self.dash_app_operator.start()


    def stop(self, *args, **kwargs):
        print('the operation agent ros node is being stopped')
        time.sleep(2)
        sys.exit(0)

    def cb_shutdown(self):
        print('the operation agent ros node is being shutdown')

    # -- callback from the GUI console
    def _console_callback(self, event, *args):
        with self.state_lock:
            pass

    def _timer_callback(self, event, *args):
        with self.state_lock:
            pass


# ---------------------------------------------------------
if __name__ == '__main__':
    capturer_config:YamlConfig = YamlConfig('../../config/capturer_config.yaml')
    rospy.init_node('shorts_capturer_agent')
    the_agent = ApplicationCoordinator(capturer_config)
    DASH_HOST = capturer_config.get('capturer.dash.host')
    DASH_PORT = capturer_config.get('capturer.dash.host')
    if capturer_config.get('capturer.dash.launch_browser', False):
        URL = f'http://{DASH_HOST}:{DASH_PORT}'
        webbrowser.open(URL)

