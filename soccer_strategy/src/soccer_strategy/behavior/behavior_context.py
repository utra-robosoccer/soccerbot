from __future__ import annotations

from soccer_strategy.behavior import Behavior
from soccer_strategy.behavior.state.balance import Balance


class BehaviorContext:
    """
    Interface to state and switching states. All code to access the state and how they switch

    It contains a reference to an instance of a Behavior subclass, which represents the current
    state.
    """

    _state = None
    """
    A reference to the current state of the BehaviorContext.
    """

    def __init__(self, sim: bool = True) -> None:
        self.sim = sim  # TODO clean up
        self.transition_to(Balance())  # Has to be last. setting context for current state

    @property
    def state(self) -> Behavior:
        return self._state  # type: ignore[return-value]

    @state.setter
    def state(self, state) -> None:
        self.transition_to(state)

    def transition_to(self, state) -> None:
        """
        The BehaviorContext allows changing the State object at runtime.
        """
        state.context = self  # Why?

        if state.ready_to_switch_to():  # TODO needed?
            print(f"BehaviorContext: Transition to {type(state).__name__}")
            self._state = state
            self._state.context = self
            self.state_action()  # TODO is this required

    """
    The BehaviorContext delegates part of its behavior to the current State object.
    """

    def state_action(self) -> None:
        self._state.action()  # type: ignore[union-attr]

    def run_state_algorithim(self) -> None:
        self._state.run_algorithim()  # type: ignore[union-attr]

    # TODO could have a fallen function
