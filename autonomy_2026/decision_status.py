from enum import Enum
class DecisionStatus(Enum):
    NEXT_POINT  = 0
    BACKTRACK   = 1
    ARRIVED     = 2