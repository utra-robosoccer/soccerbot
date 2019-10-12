import importlib
import pkgutil

from flask import Flask, Blueprint, jsonify
from flask_cors import CORS

from control_center.utils.interval_thread import IntervalTimer
from control_center.utils.utils import update_robot_status
from control_center.utils.robot_accouncement_server import RobotAnnouncementServer
from version import __version__


class CCFlaskApp(Flask):
    def __init__(self, *args, **kwargs):
        super(CCFlaskApp, self).__init__(*args, **kwargs)
        self._stopped = False
        self._status_updater_thread = None
        self._robot_announcement_server_thread = None

        self._setup_watcher()
        self._setup_robot_announcement_server()

    def _setup_robot_announcement_server(self):
        self._robot_announcement_server_thread = RobotAnnouncementServer()
        self._robot_announcement_server_thread.setDaemon(True)
        self._robot_announcement_server_thread.start()

    def _setup_watcher(self):
        self._status_updater_thread = IntervalTimer(update_robot_status, 5)
        self._status_updater_thread.setDaemon(True)
        self._status_updater_thread.start()


def create_app():
    """
    Returns device_manager's application instance.
    @return app: Flask application instance
    """
    package_name, package_path = __name__, __path__
    app = CCFlaskApp(package_name, instance_relative_config=True)
    _register_blueprints(app, package_name, package_path)
    app.config.from_object('control_center.settings')
    CORS(app, expose_headers=['Content-Disposition'])

    @app.route('/')
    def version():
        return jsonify({'version': __version__})

    return app


def _register_blueprints(app, package_name, package_path):
    """
    Register all Blueprint instances on the specified Flask application found
    in all modules for the specified package.
    :param app: the Flask application
    :param package_name: the package name
    :param package_path: the package path
    """
    rv = []
    for _, name, ispkg in pkgutil.iter_modules(package_path):
        if not ispkg:
            m = importlib.import_module('%s.%s' % (package_name, name))
            for item in dir(m):
                item = getattr(m, item)
                if isinstance(item, Blueprint):
                    app.register_blueprint(item)
                rv.append(item)
        else:
            _register_blueprints(app, '%s.%s' % (package_name, name), ['%s/%s' % (package_path[0], name)])
    return rv
