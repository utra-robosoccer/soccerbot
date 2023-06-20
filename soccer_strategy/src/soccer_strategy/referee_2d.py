import os
import subprocess
import threading
from unittest.mock import MagicMock, patch

import controller
from referee import Referee


class FakeSupervisor:
    pass


class Referee2D(Referee):
    def __init__(self):
        os.system("/bin/bash -c 'killall python3 || echo 'No Python Executables running''")
        os.system("/bin/bash -c 'killall /usr/bin/java || echo 'No Java Executables running'")

        os.environ["JAVA_HOME"] = "/usr"
        os.environ["GAME_CONTROLLER_HOME"] = os.path.dirname(os.path.realpath(__file__)) + "/../../../external/GameController/"
        controller.Supervisor = FakeSupervisor

        super().__init__()

    def list_player_solids(self, *args, **kwargs):
        pass

    def main_loop(self):
        pass

    def clean_exit(self):
        pass
