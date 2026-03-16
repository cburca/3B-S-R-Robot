from enum import Enum, auto
import time

class State(Enum):
    INIT = auto()
    FOLLOW_LINE_TO_BULLSEYE = auto()
    ALIGN_BULLSEYE = auto()
    PICKUP = auto()
    TURN_180 = auto()
    FOLLOW_LINE_TO_SAFEZONE = auto()
    DEPOSIT = auto()
    FOLLOW_LINE_TO_END = auto()
    RECOVER_LINE = auto()
    DONE = auto()
    FAULT = auto()

state = State.INIT
state_enter_t = time.monotonic()
previous_follow_state = None
carrying = False