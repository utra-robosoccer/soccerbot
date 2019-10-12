import copy
import yaml
from flask import Blueprint, request

from control_center.utils.decorators import required_online, jsonify_response
from control_center.utils.utils import (
    STATUS_OFFLINE, get_remote_file, source_ros_command, remove_self_managed_processes, get_robot_status
)
from control_center.impl.process_manager import SupervisorImpl
from control_center.settings import ROBOT_NAMES

bp = Blueprint('robots', __name__, url_prefix='/robots')


@bp.route('/', methods=['GET'])
@jsonify_response()
def list_robots():
    if not ROBOT_NAMES:
        return 'no robot list found', 400
    robots = []
    status = get_robot_status()

    for robot_ip, robot_info in ROBOT_NAMES.items():
        info = {
            'name': robot_info['name'],
            'hostname': robot_ip,
            'status': status.get(robot_ip, STATUS_OFFLINE)
        }
        robots.append(info)
    return robots


@bp.route('/<name>', methods=['GET'])
@required_online()
@jsonify_response()
def get_robot(name):
    process_status = SupervisorImpl(robot_ip=name).status_all()
    remove_self_managed_processes(process_status)
    return process_status


@bp.route('/<name>/available_launch_groups', methods=['GET'])
@required_online()
@jsonify_response()
def get_launch_groups(name):
    # Get the current config file
    robot_config = ROBOT_NAMES.get(name)
    is_error, file_data = get_remote_file(robot_config['config_path'], robot_ip=name)
    if is_error:
        return file_data, 500

    # Return all the launch configs for now
    launch_info = yaml.load(file_data)
    return launch_info.get('launch_groups', [])


@bp.route('/<name>/launch_group', methods=['POST'])
@required_online()
@jsonify_response()
def start_launch_group(name):
    try:
        group_info = request.get_json(force=True)
    except Exception:
        return 'Deserialization error', 400

    if 'group' not in group_info:
        return 'group field not found', 400

    if 'executables' in group_info and not isinstance(group_info['executables'], list):
        return 'executables should be a list', 400

    return start_launch_group_helper(group_info, name)


def start_launch_group_helper(group_info, name):
    robot_config = ROBOT_NAMES[name]
    is_error, file_data = get_remote_file(robot_config['config_path'], robot_ip=name)
    if is_error:
        return file_data, 500

    launch_info = yaml.load(file_data)
    global_env = copy.deepcopy(robot_config['ros_config'].get('env', {}))
    global_env.update(launch_info.get('env', {}))
    launch_group_def = [group for group in launch_info.get('launch_groups', []) if group['name'] == group_info['group']]
    if not launch_group_def:
        return 'invalid group name', 400
    launch_group_def = launch_group_def[0]
    if not group_info.get('executables'):
        group_info['executables'] = launch_group_def['launch_configs']

    process_list = []
    for config in launch_info.get('launch_configs', []):
        if config['name'] not in group_info['executables']:
            continue
        env = copy.deepcopy(global_env)
        env.update(config.get('env', {}))
        command = source_ros_command(config['command'])
        if config.get('monitor_node', False):
            command = command[:-1] + '  --robot={} --launch-group={} --launch-config={}"'.format(
                robot_config['name'], group_info['group'], config['name']
            )
        process_info = {'name': config['name'], 'env': env, 'command': command}
        process_list.append(process_info)
    process_status = SupervisorImpl(robot_ip=name).start_process_group(group_info['group'], process_list)
    return process_status


@bp.route('/<name>/launch_group/<group>', methods=['GET'])
@required_online()
@jsonify_response()
def get_launch_group(name, group):
    process_status = SupervisorImpl(robot_ip=name).process_group_status(group)
    return process_status


@bp.route('/<name>/launch_group/<group>', methods=['DELETE'])
@required_online()
@jsonify_response()
def stop_launch_group(name, group):
    process_status = SupervisorImpl(robot_ip=name).stop_process_group(group)
    return process_status


@bp.route('/<name>/launch_group', methods=['DELETE'])
@required_online()
@jsonify_response()
def stop_all(name):
    process_status = SupervisorImpl(robot_ip=name).stop_all()
    remove_self_managed_processes(process_status)
    return process_status


@bp.route('/<name>/launch_group/<group>/executable/<executable>', methods=['GET'])
@required_online()
@jsonify_response()
def get_executable(name, group, executable):
    process_status = SupervisorImpl(robot_ip=name).process_status(group, executable)
    return process_status


@bp.route('/<name>/launch_group/<group>/executable/<executable>', methods=['DELETE'])
@required_online()
@jsonify_response()
def stop_executable(name, group, executable):
    process_status = SupervisorImpl(robot_ip=name).stop_process(group, executable)
    return process_status
