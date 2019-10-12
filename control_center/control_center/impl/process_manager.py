# -*- coding: utf-8 -*-
import os
import logging
import paramiko
from xmlrpclib import ServerProxy, MultiCall
from ConfigParser import SafeConfigParser
from abc import ABCMeta, abstractmethod

from paramiko import SSHClient, AutoAddPolicy
from scp import SCPClient

from control_center.utils.api_response import ApiResponse
from control_center.utils.decorators import handle_supervisor_error
from control_center.settings import ROBOT_NAMES, PROCESS_CONFIG_DIR, PROCESS_LOG_DIR

logging.basicConfig()
paramiko.util.logging.getLogger().setLevel(logging.ERROR)

LOG = logging.getLogger(__name__)
ERROR_STATUS = ['BACKOFF', 'EXITED', 'FATAL', 'UNKNOWN']
RUNNING_STATUS = ['STARTING', 'RUNNING']
STOPPED_STATUS = ['STOPPING', 'STOPPED']
STATEINFO = {
    'STARTING':
        'The process is starting due to a start request.',
    'STOPPED':
        'The process has been stopped due to a stop request or has never'
        ' been started.',
    'RUNNING':
        'The process is running.',
    'STOPPING':
        'The process is stopping due to a stop request.',
    'BACKOFF':
        'The process entered the STARTING state but subsequently exited '
        'too quickly to move to the the RUNNING state',
    'EXITED':
        'The process exited from the RUNNING state (expectedly or '
        'unexpectedly).',
    'FATAL':
        'The process could not be started successfully.',
    'UNKNOWN':
        'The process is in an unknown state'
}

START_ACTION = 'succeeded'
STOP_ACTION = 'stopped'
RETRY_COUNT = 3


class ProcessManager:
    __metaclass__ = ABCMeta

    @abstractmethod
    def start_process_group(self, group, process_list):
        pass

    @abstractmethod
    def process_group_status(self, group):
        pass

    @abstractmethod
    def stop_process_group(self, group):
        pass

    @abstractmethod
    def start_process(self, group, process_info):
        pass

    @abstractmethod
    def process_status(self, group, process):
        pass

    @abstractmethod
    def stop_process(self, group, process):
        pass

    @abstractmethod
    def stop_all(self):
        pass

    @abstractmethod
    def status_all(self):
        pass


class SupervisorImpl(ProcessManager):
    def __init__(self, robot_ip='localhost', port=6001):
        self.server = ServerProxy('http://{}:{}/RPC2'.format(robot_ip, port))
        self.supervisor = self.server.supervisor
        self.robot_ip = robot_ip
        self._config = ROBOT_NAMES[robot_ip]

    def _ssh_scp_client(self):
        ssh_client = SSHClient()
        ssh_client.set_missing_host_key_policy(AutoAddPolicy())
        ssh_client.load_system_host_keys()
        if self.robot_ip == 'localhost':
            ssh_client.connect(self.robot_ip)
        else:
            ssh_client.connect(self.robot_ip, username=self._config['ssh_username'],
                               password=self._config['ssh_password'])
        scp_client = SCPClient(ssh_client.get_transport())
        return ssh_client, scp_client

    def _get_api_response(self, process_info, action=None):
        process_info['stateinfo'] = STATEINFO[process_info['statename']]
        if action == START_ACTION:
            if process_info['statename'] in ERROR_STATUS or \
                    process_info['statename'] in STOPPED_STATUS:
                return ApiResponse.serialize_error(
                    'Process in {} state'.format(process_info['statename']), data=process_info
                )
        elif action == STOP_ACTION:
            if process_info['statename'] in RUNNING_STATUS:
                return ApiResponse.serialize_error(
                    'Process in {} state'.format(process_info['statename']), data=process_info
                )
            else:
                return ApiResponse.serialize_success({'name': process_info['name']})
        return ApiResponse.serialize_success(process_info)

    def _add_process_section(self, section, process, config_parser):
        config_parser.add_section(section)
        log_file_name = process['name']
        log_path = PROCESS_LOG_DIR + log_file_name
        config_parser.set(section, 'autostart', 'false')
        config_parser.set(section, 'autorestart', 'false')
        config_parser.set(section, 'stopasgroup', 'true')
        config_parser.set(section, 'killasgroup', 'true')
        config_parser.set(section, 'stdout_logfile', log_path)
        config_parser.set(section, 'stopsignal', 'INT')  # todo: only for rosmon
        config_parser.set(section, 'redirect_stderr', 'true')
        config_parser.set(section, 'stdout_logfile_maxbytes', '1MB')
        config_parser.set(section, 'command', process['command'])

        if process.get('user'):
            config_parser.set(section, 'user', process['user'])

        environment = ''
        for key, value in process['env'].iteritems():
            environment = '{},{}="{}"'.format(environment, key, value)
        environment = environment[1:]
        config_parser.set(section, 'environment', environment)

    def _write_sections(self, config_parser, fp):
        for section in config_parser._sections:
            fp.write("[%s]\n" % section)
            for (key, value) in config_parser._sections[section].items():
                if key == "__name__":
                    continue
                if (value is not None) or (config_parser._optcre == config_parser.OPTCRE):
                    key = " = ".join((key, str(value).replace('\n', '\n\t')))
                fp.write("%s\n" % key)
            fp.write("\n")

    def _write_to_file_and_validate(self, config_parser, group):
        filename = group + '.conf'
        local_file = '/tmp/' + filename
        remote_new_file = PROCESS_CONFIG_DIR + filename
        remote_tmp_file = remote_new_file + '.bak'

        retries_performed = 0
        success = False
        while retries_performed < RETRY_COUNT:
            fp = open(local_file, 'w')
            self._write_sections(config_parser, fp)
            fp.flush()
            os.fsync(fp.fileno())
            fp.close()
            ssh, scp = None, None
            try:
                ssh, scp = self._ssh_scp_client()
                scp.put(local_file, remote_tmp_file)
                stdin, stdout, stderr = ssh.exec_command('mv {} {}'.format(remote_tmp_file, remote_new_file))
                exit_status = stdout.channel.recv_exit_status()  # Blocking call
                if exit_status != 0:
                    LOG.error('Supervisor config rm error, {}'.format(exit_status))
                    break
                success = True
                break
            except Exception as e:
                retries_performed += 1
                LOG.error('Supervisor config write error, {}'.format(e.message))
            finally:
                if scp:
                    scp.close()
                if ssh:
                    ssh.close()

        if not success:
            return 'Config write failed for the deployment'
        return ''

    def _write_to_file_bulk(self, config_parser_dict):
        local_files = []
        commands = []
        for process_name, parser in config_parser_dict.iteritems():
            filename = process_name + '.conf'
            local_file = '/tmp/' + filename + '.bak'
            remote_new_file = PROCESS_CONFIG_DIR + filename
            remote_tmp_file = remote_new_file + '.bak'
            fp = open(local_file, 'w')
            self._write_sections(parser, fp)
            fp.flush()
            os.fsync(fp.fileno())
            fp.close()
            local_files.append(local_file)
            commands.append('mv {} {}'.format(remote_tmp_file, remote_new_file))

        retries_performed = 0
        success = False
        ssh_cmd = ';'.join(commands)
        while retries_performed < RETRY_COUNT:
            ssh, scp = None, None
            try:
                ssh, scp = self._ssh_scp_client()
                scp.put(local_files, PROCESS_CONFIG_DIR)
                stdin, stdout, stderr = ssh.exec_command(ssh_cmd)
                exit_status = stdout.channel.recv_exit_status()  # Blocking call
                if exit_status != 0:
                    LOG.error('Supervisor config rm error, {}'.format(exit_status))
                    break
                success = True
                break
            except Exception as e:
                retries_performed += 1
                LOG.error('Supervisor config write error, {}'.format(e.message))
            finally:
                if scp:
                    scp.close()
                if ssh:
                    ssh.close()

        if not success:
            return 'Config write failed for the deployment'
        return ''

    def _purge_config(self, group):
        ssh = None
        dep_file = PROCESS_CONFIG_DIR + group + '.conf'
        try:
            ssh, _ = self._ssh_scp_client()
            stdin, stdout, stderr = ssh.exec_command('rm {}'.format(dep_file))
            exit_status = stdout.channel.recv_exit_status()  # Blocking call
            if exit_status != 0:
                LOG.error('Supervisor config rm error, {}'.format(exit_status))
        except Exception as e:
            LOG.error('Supervisor config rm error, {}'.format(e.message))
        finally:
            if ssh:
                ssh.close()

    def _purge_all_config(self):
        folder = PROCESS_CONFIG_DIR
        ssh = None
        try:
            ssh, _ = self._ssh_scp_client()
            stdin, stdout, stderr = ssh.exec_command('rm {}*'.format(folder))
            exit_status = stdout.channel.recv_exit_status()  # Blocking call
            if exit_status != 0:
                LOG.error('Supervisor config rm error, {}'.format(exit_status))
        except Exception as e:
            LOG.error('Supervisor config rm error, {}'.format(e.message))
        finally:
            if ssh:
                ssh.close()

    def _process_name(self, group, exe_name):
        return 'group-{}-exe-{}'.format(group, exe_name)

    def _parse_group_and_exe(self, process_name):
        if not process_name.startswith('group-'):
            return None, None
        process_name = process_name[6:]
        substrings = process_name.split('-exe-')
        return substrings

    def _get_group_status(self, group, action=None):
        all_info = self.supervisor.getAllProcessInfo()
        group_status = ApiResponse.serialize_success({})
        for info in all_info:
            exe_group, exe_name = self._parse_group_and_exe(info['name'])
            if exe_group == group:
                group_status[exe_name] = self._get_api_response(info, action=action)
                if ApiResponse.is_error(group_status[exe_name]):
                    ApiResponse.serialize_error('Some processes in error state', group_status)

        return {group: group_status}

    def _get_all_status(self, action=None):
        all_info = self.supervisor.getAllProcessInfo()
        all_status = dict()
        for info in all_info:
            group, name = self._parse_group_and_exe(info['name'])
            if not group:
                group = info['group']
            group_status = all_status.get(group, ApiResponse.serialize_success({}))
            group_status[name] = self._get_api_response(info, action=action)
            if ApiResponse.is_error(group_status[name]):
                ApiResponse.serialize_error('Some processes in error state', group_status)
            all_status[group] = group_status

        return all_status

    def _add_process_block(self, process_info):
        config_parser = SafeConfigParser()
        section = 'program:{}'.format(process_info['name'])
        self._add_process_section(section, process_info, config_parser)
        return self._write_to_file_and_validate(config_parser, process_info['name'])

    def _get_process_config_parser(self, group, process_info):
        process_name = self._process_name(group, process_info['name'])
        process_info['name'] = process_name
        config_parser = SafeConfigParser()
        section = 'program:{}'.format(process_info['name'])
        self._add_process_section(section, process_info, config_parser)
        return config_parser

    def _multicall_start_processes(self, process_list):
        updated_config = self.supervisor.reloadConfig()
        added = updated_config[0][0]

        for process in process_list:
            process_name = process['name']
            if process_name in added:
                self.server.supervisor.addProcessGroup(process_name)

        multicall = MultiCall(self.server)

        for process in process_list:
            process_name = process['name']
            multicall.supervisor.startProcess(process_name)
            multicall.supervisor.getProcessInfo(process_name)
        return multicall()

    @handle_supervisor_error
    def start_process(self, group, process_info):
        process_name = self._process_name(group, process_info['name'])
        process_info['name'] = process_name
        error = self._add_process_block(process_info)
        if error:
            return ApiResponse.serialize_error(error)
        updated_config = self.supervisor.reloadConfig()
        added = updated_config[0][0]
        if process_name in added:
            self.supervisor.addProcessGroup(process_name)

        self.supervisor.startProcess(process_name)
        process_info = self.supervisor.getProcessInfo(process_name)
        return self._get_api_response(process_info, action=START_ACTION)

    @handle_supervisor_error
    def process_status(self, group, name):
        process_info = self.supervisor.getProcessInfo(self._process_name(group, name))
        return self._get_api_response(process_info, action=START_ACTION)

    @handle_supervisor_error
    def stop_process(self, group, name):
        process_name = self._process_name(group, name)
        process_info = self.supervisor.getProcessInfo(process_name)
        if not process_info:
            return {}
        if process_info['statename'] in RUNNING_STATUS:
            self.supervisor.stopProcess(process_name)
        process_info = self.supervisor.getProcessInfo(process_name)
        response = self._get_api_response(process_info, action=STOP_ACTION)
        self._purge_config(process_name)
        self.supervisor.reloadConfig()
        self.supervisor.removeProcessGroup(process_name)
        return response

    @handle_supervisor_error
    def start_process_group(self, group, process_list):
        response = {group: ApiResponse.serialize_success({})}
        config_dict = {}
        for process in process_list:
            config_dict[process['name']] = self._get_process_config_parser(group, process)
        error = self._write_to_file_bulk(config_dict)
        if error:
            return ApiResponse.serialize_error(error)

        result = self._multicall_start_processes(process_list)

        for process_status in result.results[1::2]:
            if 'name' not in process_status:
                continue
            else:
                process_name = self._parse_group_and_exe(process_status['name'])[1]
                response[group][process_name] = self._get_api_response(process_status, action=START_ACTION)
                if ApiResponse.is_error(response[group][process_name]):
                    ApiResponse.serialize_error('Some processes in error state', response[group])
        return response

    @handle_supervisor_error
    def process_group_status(self, group):
        return self._get_group_status(group, action=START_ACTION)

    @handle_supervisor_error
    def stop_process_group(self, group):
        # return if process group does not exists
        group_status = self._get_group_status(group)
        response = {group: ApiResponse.serialize_success({})}
        for process in group_status[group]:
            if process == 'status':
                continue
            process_output = self.stop_process(group, process)
            response[process] = process_output
        return response

    @handle_supervisor_error
    def stop_all(self):
        self.supervisor.stopAllProcesses()
        response = self._get_all_status(action=STOP_ACTION)
        self._purge_all_config()
        self.supervisor.restart()
        return response

    @handle_supervisor_error
    def status_all(self):
        return self._get_all_status(action=START_ACTION)
