from __future__ import annotations

from abc import ABC, abstractmethod


class Behavior(ABC):
    """
    The action a state should do like Arm, TakeOff. All state specific code

    It contains a reference to an instance of a BehaviorContext class
    This reference allows for each behavior to switch to another behavior.
    """

    @property
    def context(self):
        # link to transition state
        return self._context

    @context.setter
    def context(self, context) -> None:
        self._context = context

    @abstractmethod
    def action(self) -> None:  # TODO need to rethink this action
        # Updating drone status and performing actions for that state
        pass

    @abstractmethod
    def run_algorithim(self) -> None:
        # Condition to check based on drone state
        pass

    @abstractmethod
    def ready_to_switch_to(self) -> bool:
        return True

    @property
    def state(self) -> Behavior:
        return self._context.state  # type: ignore[no-any-return]

    @state.setter
    def state(self, state) -> None:
        self._context.state = state
