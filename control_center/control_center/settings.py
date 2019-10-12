from os import getenv

# Fixed Parameters
PROCESS_CONFIG_DIR = '~/sootballs/supervisor/'
PROCESS_LOG_DIR = '/var/log/sootballs/supervisor/'

# Different Per robot
CONFIG_PATH = '/config.yaml'
ROS_CONFIG = {
    'env': {
        'ROS_DOMAIN_ID': getenv('ROS_DOMAIN_ID', 0),
        'USE_CLOUD_BRIDGE': getenv('USE_CLOUD_BRIDGE', 'false'),
        'ROBOT': getenv('ROBOT', 'zombie'),
        'DISABLE_BRIDGE': getenv('DISABLE_BRIDGE', 'false'),
        'AMQP_HOST': getenv('AMQP_HOST', 'rabbit'),
        'IMS_URL': getenv('IMS_URL', 'http://ims:8002'),
        'MAP_URL': getenv('MAP_URL', 'http://map_server:6789'),
        'ROSBRIDGE_URL': getenv('ROSBRIDGE_URL', 'localhost:9092'),
        'SOOTBALLS_MAP': getenv('SOOTBALLS_MAP'),
        'FAST_SIMULATION': getenv('FAST_SIMULATION', 'false'),
        'SIMULATION': getenv('SIMULATION', 'false'),
        'USE_LOCAL_MAP': getenv('USE_LOCAL_MAP', 'false'),
        'UPDATE_RATE': getenv('UPDATE_RATE', 100),
        'STEP_SIZE': getenv('STEP_SIZE', 0.01),
        'ROSBRIDGE_ONLY': getenv('ROSBRIDGE_ONLY', 'false'),
        'UI_ONLY': getenv('UI_ONLY', 'false'),
        'ROS_MASTER_URI': getenv('ROS_MASTER_URI', 'http://localhost:11311')
    }
}
SSH_USERNAME = 'rr'
SSH_PASSWORD = 'airborne'

# Dictionary of all robots
ROBOT_NAMES = {}
