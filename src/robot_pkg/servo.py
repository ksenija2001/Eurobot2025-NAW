from dataclasses import dataclass
from enum import Enum
import can
from queue import Queue

@dataclass
class ServoPositions():
    TOP : int
    MIDDLE : int
    BOTTOM : int
    
    def __init__(self, top=None, middle=None, bottom=None):
        self.TOP = top
        self.MIDDLE = middle
        self.BOTTOM = bottom

class Servo(Enum):
    def __init__(self, id:int, positions:ServoPositions):
        self.id = id
        self.positions = positions

    RIGHT_VACUUM_LIFT = 1, ServoPositions(top=  0,             bottom=300) 
    RIGHT_VACUUM      = 2, ServoPositions(top=240, middle=150, bottom= 60)

    LEFT_VACUUM_LIFT  = 3, ServoPositions(top=300,             bottom=  0) 
    LEFT_VACUUM       = 4, ServoPositions(top= 60, middle=150, bottom=240)

    RIGHT_GRIP_LIFT   = 5, ServoPositions(top=  0,             bottom=300)
    LEFT_GRIP_LIFT    = 6, ServoPositions(top=300,             bottom=  0)

    CENTER_SWING      = 7, ServoPositions(top=240,             bottom=150)
    CENTER_LIFT       = 8, ServoPositions(top=  0,             bottom=242)

    BACK_RIGHT_LIFT   = 9, ServoPositions(top=240,             bottom=150)
    BACK_LEFT_LIFT    =10, ServoPositions(top=  0,             bottom=242)


class ServoMoving:

    def __init__(self, can_queue: Queue):
        self.can_queue = can_queue
    
    def send(self, ids:list, positions:list):
        pass


if __name__ == "__main__":
    print(Servo.RIGHT_VACUUM_LIFT.positions.BOTTOM)
    print(Servo.LEFT_VACUUM_LIFT.positions.BOTTOM)

