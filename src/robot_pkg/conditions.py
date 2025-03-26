from enum import Enum
from data import Variables
import time


class ConditionType(Enum):
    TIMEOUT    = 0
    TIME       = 1
    POSITION   = 2
    END_SWITCH = 3
    CINCH      = 4
    DETECTION  = 5
    SERVO      = 6

def timeout(timeout, args) -> bool:
    start_time = args[0]
    curr_time  = args[1]
    if curr_time - start_time >= timeout:
        return True
    return False

def match_time(_time, args) -> bool:
    if Variables.match_start_time  + _time < args[1]:
        return True
    return False

def position(tmp, args) -> bool:
    curr_type_of_movement = args[2]
    if curr_type_of_movement == 0:
        return True
    return False

def cinch(tmp, args) -> bool:
    curr_cinch = args[3]
    if not curr_cinch:
        Variables.match_start_time = time.time()
        return True
    return False

def servo(tmp, args) -> bool:
    curr_pos = args[4]
    if curr_pos:
        return True
    return False


conditions = {
    ConditionType.TIMEOUT : timeout,
    ConditionType.TIME    : match_time, 
    ConditionType.POSITION: position,
    ConditionType.CINCH   : cinch,
    ConditionType.SERVO   : servo,
}

class Condition:

    def __init__(self, args:tuple): #_type, step_ID=None, value=None)
        self.type = args[0]
        self.value = None
        self.ID = None

        if len(args) > 1:
            self.ID = args[1]
        if len(args) > 2:
            self.value = args[2]

    def check(self, args:list):
        if conditions[self.type](self.value, args):
            return self.ID
        return False
    
    def __repr__(self):
        return str(self.type) + " " + str(self.ID) + " " + str(self.value)

if __name__ == "__main__":
    cond = Condition((ConditionType.POSITION,))
    print(cond)