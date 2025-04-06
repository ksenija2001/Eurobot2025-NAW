from enum import Enum
from robot_pkg.data import Variables
import time


class ConditionType(Enum):
    TIMEOUT    = 0      # Wait for defined amount of time, delay
    TIME       = 1      # Check if match time has run out
    POSITION   = 2      # Wait for movement to finish
    END_SWITCH = 3      
    CINCH      = 4      # Wait for cinch to be pulled 
    DETECTION  = 5      # React to a detection
    SERVO      = 6      # Wait for servo to finish moving
    SERVO_POSITION = 7  # Check before step if servo is in last set position

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
    move_done = args[2]
    if move_done:
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

    def __init__(self, args:tuple): 
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