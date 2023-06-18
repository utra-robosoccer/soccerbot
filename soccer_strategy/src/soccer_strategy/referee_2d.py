from unittest.mock import MagicMock, patch

import controller
from referee import Referee


class FakeSupervisor:
    pass


class Referee2D(Referee):
    def __init__(self):
        controller.Supervisor = FakeSupervisor

        super().__init__()

    pass
