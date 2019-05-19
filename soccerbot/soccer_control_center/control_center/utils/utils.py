import urllib2
from subprocess import check_output, CalledProcessError, STDOUT
import logging
from urlparse import urljoin

from control_center.settings import ROBOT_NAMES

LOG = logging.getLogger(__name__)
LOG.setLevel(logging.getLevelName('WARNING'))
logging.getLogger("werkzeug").setLevel(logging.WARNING)
STATUS_ONLINE = 'online'
STATUS_OFFLINE = 'offline'
self_managed = ['config_server']
__ROBOT_STATUS__ = {}


def update_robot_status():
    # using settings instead of current_app as app_context is not working in a thread
    for robot in ROBOT_NAMES:
        status = STATUS_ONLINE if is_online(robot['hostname']) else STATUS_OFFLINE
        global __ROBOT_STATUS__
        __ROBOT_STATUS__[robot['hostname']] = status


def get_robot_status():
    global __ROBOT_STATUS__
    return __ROBOT_STATUS__


def is_online(host):
    try:
        check_output(['ping', '-c', '1', '-t', '3', host], stderr=STDOUT)
    except CalledProcessError as e:
        LOG.info('robot {} not online, output:{}'.format(host, e.output))
        return False
    except Exception as e:
        LOG.error('Exception in is_online:{}'.format(e.message))
        return False

    return True


def get_remote_file(path, hostname='localhost', port=8000):
    error = True
    try:
        url = urljoin('http://{}:{}/'.format(hostname, port), path)
        response = urllib2.urlopen(url)
        return False, response.read()
    except urllib2.URLError as e:
        LOG.error('Exception in getting remote file for {}: {}'.format(hostname, e.message))
        return error, e.message


def source_ros_command(command, ros2, ros_config):
    if ros2:
        return 'bash -c . /etc/profile.d/soccer.sh && " . ${{ROS2_WS_PATH}}/install/setup.bash && {}"'.format(command)
    return 'bash -c " . /etc/profile.d/soccer.sh && . /opt/ros/${{ROS1_DISTRO}}/setup.bash && '\
        '. ${{ROS1_WS_PATH}}/devel/setup.bash && {}"'.format(command)


def remove_self_managed_processes(process_status):
    for name in self_managed:
        process_status.pop(name, None)


def init_robot_config(app_config):
    for robot in app_config['ROBOT_NAMES']:
        config = {
            'name': robot['name'],
            'config_path': robot.get('config_path', app_config['CONFIG_PATH']),
            'process_config_dir': robot.get('process_config_dir', app_config['PROCESS_CONFIG_DIR']),
            'process_log_dir': robot.get('process_log_dir', app_config['PROCESS_LOG_DIR']),
            'ros_config': robot.get('ros_config', app_config['ROS_CONFIG']),
            'ssh_username': robot.get('ssh_username', app_config['SSH_USERNAME']),
            'ssh_password': robot.get('ssh_password', app_config['SSH_PASSWORD']),
        }
        app_config[robot['hostname']] = config
