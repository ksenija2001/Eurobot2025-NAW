import time

from robot_pkg.main import can_handler, log_handler
from robot_pkg.conditions import Condition
from robot_pkg.servo import Servo
from robot_pkg.move import Move
from robot_pkg.io import I_O

class Step:
    def __init__(self, ID, movement:Move, outputs:list[I_O], servos:list[Servo], conditions:list[tuple], points):
        self.ID = ID
        self.movement = movement
        self.outputs = outputs
        self.servos = servos
        self.conditions = [Condition(cond) for cond in conditions]
        self.points = points

    def step(self):
        if self.movement is not None:
            print(f"Executing movement {self.movement._type}")
            self.movement._execute()

        for output in self.actuation:
            print(f"Executing actuator {output._type}")
            output._execute()

        for servo in self.servos:
            print(f"Executing servo {servo._type}")
            servo._execute()

        # All executed servos are started at the same time
        if len(self.servos) > 0:
            Servo.send_positions()


if __name__ == "__main__":

    can_handler.start_threads()

    valve = I_O.Valve(0)
    valve._execute()

    time.sleep(1)

    can_handler.stop_threads()







    
