import urllib2
from subprocess import check_output, CalledProcessError, STDOUT
import logging
from urlparse import urljoin
from socket import gethostname
import warnings
from os import getenv
from copy import deepcopy

from control_center.settings import ROBOT_NAMES, CONFIG_PATH, ROS_CONFIG, \
    SSH_USERNAME, SSH_PASSWORD
from control_center.utils.robot_accouncement_server import SOCKET_QUEUE

LOG = logging.getLogger(__name__)
LOG.setLevel(logging.getLevelName('WARNING'))
logging.getLogger("werkzeug").setLevel(logging.WARNING)
warnings.simplefilter("ignore")
warnings.warn("deprecated", DeprecationWarning)

STATUS_ONLINE = 'online'
STATUS_OFFLINE = 'offline'
self_managed = ['config_server']
__ROBOT_STATUS__ = {}


def update_robot_status():
    # check if any new robots appeared

    while not SOCKET_QUEUE.empty():
        new_robot = SOCKET_QUEUE.get()

        is_new_robot = True
        for robot_ip in ROBOT_NAMES:
            if robot_ip == new_robot[1][0]:
                is_new_robot = False
                continue

        if not is_new_robot:
            continue

        # TODO get these parameters directly from lighttpd file server
        robot_info = {
            'name': new_robot[0].replace('-', '_').lower(),
            'config_path': CONFIG_PATH,
            'ros_config': deepcopy(ROS_CONFIG),
            'ssh_username': SSH_USERNAME,
            'ssh_password': SSH_PASSWORD,
        }

        # For testing purposes when the ros master uri is different
        if new_robot[0] == gethostname():
            robot_info['ros_config']['env']['ROS_MASTER_URI'] = getenv('ROS_MASTER_URI',
                                                                       'http://' + gethostname() + ':11311')

        if new_robot[0] == gethostname():
            ROBOT_NAMES.update({'localhost': robot_info})
        else:
            ROBOT_NAMES.update({new_robot[1][0]: robot_info})

    # using settings instead of current_app as app_context is not working in a thread
    for robot_ip in ROBOT_NAMES:
        status = STATUS_ONLINE if is_online(robot_ip) else STATUS_OFFLINE
        global __ROBOT_STATUS__
        __ROBOT_STATUS__[robot_ip] = status


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


def get_remote_file(path, robot_ip='localhost', port=8000):
    error = True
    try:
        url = urljoin('http://{}:{}/'.format(robot_ip, port), path)
        response = urllib2.urlopen(url)
        return False, response.read()
    except urllib2.URLError as e:
        LOG.error('Exception in getting remote file for {}: {}'.format(robot_ip, e.message))
        return error, e.message


def source_ros_command(command):
    return 'bash -E -i -c " . /etc/profile.d/sootball.sh && . /opt/ros/${{ROS1_DISTRO}}/setup.bash && ' \
           '. ${{ROS1_WS_PATH}}/devel/setup.bash && {}"'.format(command)


def remove_self_managed_processes(process_status):
    for name in self_managed:
        process_status.pop(name, None)
