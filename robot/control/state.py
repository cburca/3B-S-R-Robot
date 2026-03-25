from enum import Enum, auto
from dataclasses import dataclass, field
import time


class State(Enum):
    WAIT_TO_START = auto()
    SEARCHING = auto()
    BULLSEYE_LINEUP = auto()
    RETRIEVAL = auto()
    FIND_SAFETY = auto()
    SAFETY_LINEUP = auto()
    DEPOSITION = auto()
    RETURN = auto()
    DONE = auto()
    FAULT = auto()


NOMINAL_FLOW = {
    State.WAIT_TO_START: State.SEARCHING,
    State.SEARCHING: State.BULLSEYE_LINEUP,
    State.BULLSEYE_LINEUP: State.RETRIEVAL,
    State.RETRIEVAL: State.FIND_SAFETY,
    State.FIND_SAFETY: State.DEPOSITION,
    State.DEPOSITION: State.RETURN,
    State.RETURN: State.DONE,
}


@dataclass
class RobotStateMachine:
    state: State = State.WAIT_TO_START
    prev_state: State | None = None
    entered_at: float = field(default_factory=time.monotonic)

    def transition_to(self, new_state: State) -> None:
        if new_state == self.state:
            return
        self.prev_state = self.state
        self.state = new_state
        self.entered_at = time.monotonic()

    def time_in_state(self) -> float:
        return time.monotonic() - self.entered_at

    def next(self) -> None:
        if self.state not in NOMINAL_FLOW:
            raise ValueError(f"No nominal next state for {self.state}")
        self.transition_to(NOMINAL_FLOW[self.state])