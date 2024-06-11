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
import sys, os, signal, time, threading, webbrowser, subprocess, traceback, croniter
from os.path import expanduser
from datetime import datetime
# ros modules
import rospy, message_filters, actionlib, rospkg
# project modules
from tools.yaml_tools import YamlConfig
from tools.logging_tools import logger
import tools.hash_tools as hash_tools
import tools.file_tools as file_tools
from tools.google_drive_tools import GoogleDriveProxy
import capturer.model as model
from capturer.model import DAO, ASSET_FILE_MANAGER, STATE, CONFIG, SystemStates, CapturerDAO
from capturer.web.dashapp_top import DashAppTop
from capturer.web.upload_helper import process_excel_file

class ApplicationCoordinator(object):
    def __init__(self):
        # save the config
        self.yaml_config:YamlConfig = CONFIG
        # create lock for synchronization
        self.state_lock = threading.RLock()

        # create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)
        rospy.on_shutdown(self.cb_shutdown)

        # prepare operation mode
        self.operation_mode = rospy.get_param('mode')
        if self.operation_mode is None or self.operation_mode == "":
            self.operation_mode = CONFIG.get('capturer.mode', 'web')
        ASSET_FILE_MANAGER.record_event(f'Capturer operation starts')
        ASSET_FILE_MANAGER.record_event(f'The operation model is "{self.operation_mode}"')
        # setup temp folder for bagfile initial location
        self.temp_folder = os.path.join(expanduser("~"), '.bagfiles')
        if not self.setup_temp_folder(self.temp_folder):
            logger.warning(f'BagfileCapturer (__init__): unable to create or setup temp folder ("{self.temp_folder}")')
            sys.exit(0)  
    
        # setup pulling of setting excel file
        self.pull_gdrive_fileid = CONFIG.get('capturer.pull.gdrive.fileid', None)
        if self.pull_gdrive_fileid is not None:
            # pull the schedule initially if the schedule fileid is given
            self.retrieve_schedule()
            # setup the schedule for pulling schedule
            schedule_spec = CONFIG.get('capturer.pull.cron.schedule', '0 * * * *')
            self.the_croniter = croniter.croniter(schedule_spec, datetime.now())
            if croniter.croniter.is_valid(schedule_spec):
                self.next_pull_datetime = self.the_croniter.get_next(datetime)
                ASSET_FILE_MANAGER.record_event(f'The schedule for pulling setting is "{self.the_croniter.expressions}" and the next pull is at {self.next_pull_datetime}') 
                self.pull_excel_hash = None
            else:
                self.pull_gdrive_fileid = None
        # register the callbacks (using the dash callback instead of the ros callback reduces the flickering on the web GUI)
        model.CALLBACK_MANAGER.set_listener(model.CallbackTypes.TIMER, self._timer_callback)
        # create the dash application   
        if self.operation_mode == 'web':
            self.dash_app_operator = DashAppTop()
            self.dash_app_operator.start()
        elif self.operation_mode == 'headless':
            self.timer = rospy.Timer(rospy.Duration(CONFIG.get('capturer.system.timer', 1.0)), self._timer_callback)
            rospy.spin()
        else:
            logger.warning(f'BagfileCapturer (__init__): invalid capturer.mode ("{self.operation_mode}") in config')
            sys.exit(0)

    def stop(self, *args, **kwargs):
        ASSET_FILE_MANAGER.record_event('BagfileCapturer: ros node is being stopped')
        time.sleep(2)
        sys.exit(0)

    def cb_shutdown(self):
        ASSET_FILE_MANAGER.record_event('BagfileCapturer: ros node is being shutdown')

    # -- callback from the GUI console
    def _console_callback(self, event, *args):
        with self.state_lock:
            pass

    def _timer_callback(self, event, *args):
        with self.state_lock:
            state = STATE.get()
            # the state transition machine
            if state == SystemStates.READY:
                # check the database if the time has reached for any scheduled capture
                schedule = DAO.query_schedule(CapturerDAO.ScheduleStates.PENDING)                
                if schedule is not None:
                    # a schedule capture is to be executed
                    STATE.update(SystemStates.CAPTURING)
                    try:
                        DAO.update_schedule_status(schedule['timestamp'], CapturerDAO.ScheduleStates.RECORDING)
                    except Exception as e:
                        logger.error(e)
                    # execute the scheduled capture
                    self.the_thread = self.exec_scheduled_capture(schedule)
                else: 
                    # if there is no scheduled capture, then check if it is time for pulling setting file                 
                    if self.pull_gdrive_fileid is not None:
                        if datetime.now() >= self.next_pull_datetime:
                            # update the next pull time
                            self.next_pull_datetime = self.the_croniter.get_next(datetime)
                            # if the time is reached to pull the system settings excel file
                            STATE.update(SystemStates.PULL_CONFIG) 
                            self.retrieve_schedule()
                            STATE.update(SystemStates.READY)
            elif state == SystemStates.CAPTURING:
                pass

    # Retrieve schedule from google drive
    def retrieve_schedule(self):
        try:
            # retrieve the system settings excel file
            excel_bytes = GoogleDriveProxy.download_file_with_fileid(self.pull_gdrive_fileid)
            if excel_bytes is not None:     
                excel_hash = hash_tools.hash_bytearray(excel_bytes)
                if not hasattr(self, 'pull_excel_hash') or self.pull_excel_hash is None or self.pull_excel_hash != excel_hash:
                    self.pull_excel_hash = excel_hash
                    error_list = process_excel_file(excel_bytes, ['Schedule', 'Rostopics', 'Accounts'], 'Setting.xlsx', None)  
                    DAO.update_expired_schedule()
                    ASSET_FILE_MANAGER.record_event(f'BagfileCapturer (pull_setting_file): pulled and update settings')
                else:
                    ASSET_FILE_MANAGER.record_event(f'BagfileCapturer (pull_setting_file): pulled the unchanged settings and ignored')
            else:
                ASSET_FILE_MANAGER.record_event(f'BagfileCapturer (pull_setting_file): unable to pull settings')
        except Exception as e:
            ASSET_FILE_MANAGER.record_event(f'BagfileCapturer (pull_setting_file): unable to pull settings due to error {e}')

    # Setup a temp folder for capturing bagfile before moving to the target location
    def setup_temp_folder(self, temp_folder):
        try:
            os.makedirs(temp_folder, exist_ok=True)
            file_tools.remove_folder_content(temp_folder)
            return True
        except Exception as e:
            return False

    # Execute a scheduled capture of bagfile using a subprocess
    def exec_scheduled_capture(self, schedule:dict):
        # compute the parameters for saving the bagfiles
        output_folder = ASSET_FILE_MANAGER.get_output_bagfiles_folder(schedule.get('folder', ''))
        prefix = schedule.get('prefix', '')
        filename = f'{prefix}{self.generate_bagfile_prefix(schedule["timestamp"])}-{DAO.count_bagfiles() + 1}.bag'
        full_path = os.path.join(output_folder, filename)
        duration = schedule['duration']
        topics_list = DAO.list_rostopic_names() 
        schedule['topics_list'] = ' '.join(topics_list)
        schedule['filename'] = filename
        schedule['full_path'] = full_path
        temp_path = os.path.join(self.temp_folder, filename)
        schedule['temp_path'] = temp_path
        # compose the exec script based on the parameters
        the_exec_script = ['/opt/ros/noetic/bin/rosbag', 'record', f'--duration={duration}', f'--output-name={temp_path}']
        the_exec_script.extend(topics_list)
        schedule['exec_script'] = " ".join(the_exec_script)
        STATE.set_var('capture_info', schedule)
        ASSET_FILE_MANAGER.record_event(f'BagfileCapturer (exec_scheduled_capture): start rosbag record\n{schedule["exec_script"]}')
        # execute the capture exec script using a callback mechanism that calls a function when the recording is complete
        the_thread = self.exec_subprocess_and_callback(self.cb_rosbag_record_completed, the_exec_script)
        return the_thread

    # the function that starts a thread for running the subprocess that runs the given exec script
    @staticmethod
    def exec_subprocess_and_callback(callback_fn, popen_args):
        def run_in_thread(callback_fn, popen_args):
            # execute the script in a thread
            try:
                # proc = subprocess.Popen(popen_args, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
                proc = subprocess.Popen(popen_args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)            
                proc.wait()
                errors = []
                for line in proc.stderr:
                    errors.append(line.decode())  
                callback_fn(''.join(errors))
                sys.exit(1)
            except Exception as e:
                logger.warning(traceback.format_exc())
        # create and start a thread running the above function
        thread = threading.Thread(target=run_in_thread, args=(callback_fn, popen_args))
        thread.start()
        return thread 

    @staticmethod
    def generate_bagfile_prefix(timestamp):
        datestr = datetime.fromtimestamp(timestamp).strftime('%Y%m%d-%H%M%S')
        return datestr

    # the callback function when the recording is complete
    def cb_rosbag_record_completed(self, error):
        if error is not None and len(error) > 0:
            # if there was an error, record the failure status
            ASSET_FILE_MANAGER.record_event(f'BagfileCapturer (cb_rosbag_play_completed): failed in rosbag record: {error}') 
            DAO.update_schedule_status(schedule['timestamp'], CapturerDAO.ScheduleStates.FAILED)           
        else:
            # record the success status and the output bagfile information
            ASSET_FILE_MANAGER.record_event(f'BagfileCapturer (cb_rosbag_play_completed): finished rosbag record') 
            schedule = STATE.get_var('capture_info')
            # if there was no error, copy the bagfile to the correct place
            os.rename(schedule['temp_path'], schedule['full_path'])
            file_size = os.stat(schedule['full_path'])
            DAO.add_bagfiles(schedule['timestamp'], schedule['duration'], schedule.get('folder', None), schedule['filename'], schedule['full_path'], file_size.st_size) 
            DAO.update_schedule_status(schedule['timestamp'], CapturerDAO.ScheduleStates.SUCCESS)
        # change the state back to READY
        STATE.update(SystemStates.READY)

# ---------------------------------------------------------
# The main program for running the bagfiles capturer
if __name__ == '__main__':
    rospy.init_node('bagfiles_capturer_agent')
    the_agent = ApplicationCoordinator()
    DASH_HOST = CONFIG.get('capturer.web.host')
    DASH_PORT = CONFIG.get('capturer.web.host')
    if CONFIG.get('capturer.web.launch_browser', False):
        URL = f'http://{DASH_HOST}:{DASH_PORT}'
        webbrowser.open(URL)

