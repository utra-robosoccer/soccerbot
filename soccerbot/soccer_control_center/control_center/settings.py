from os import getenv

# assumed that all the robots have same settings
CONFIG_PATH = '/config.yaml'
PROCESS_CONFIG_DIR = '/home/rr/soccer/supervisor/'
PROCESS_LOG_DIR = '/var/log/soccer/supervisor/'
ROS_CONFIG = {
    'env': {
    }
}
SSH_USERNAME = 'rr'
SSH_PASSWORD = 'airborne'
ROBOT_NAMES = [
    {
        'name': 'This-Computer',
        'hostname': 'localhost',
        'config_path': '/config.yaml',
        'process_config_dir': '~/soccer/supervisor/',
        'process_log_dir': '/var/log/soccer/supervisor/',
        'ros_config': {
            'env': {
                'ROS_DOMAIN_ID': getenv('ROS_DOMAIN_ID')
            }
        }
    },
    {
        'name': 'robot1',
        'hostname': 'robot1.local',
        'ros_config': {
        }
    },
]
