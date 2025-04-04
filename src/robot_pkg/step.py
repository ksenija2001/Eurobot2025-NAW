import time

from robot_pkg.main import can_handler, log_handler
from robot_pkg.conditions import Condition
from robot_pkg.servo import Servo
from robot_pkg.move import Move, Position
from robot_pkg.in_out import I_O

class Step:
    def __init__(self, ID, movement:Move, outputs:list[I_O], servos:list[Servo], conditions:list[tuple], points):
        self.ID = ID
        self.movement = movement
        self.outputs = outputs
        self.servos = servos
        self.conditions = [Condition(cond) for cond in conditions]
        self.points = points

    def move(self):
        if self.movement is not None:
            print(f"Executing movement {self.movement._type}")
            self.movement._execute()

    def output(self, curr_pose:Position=Position()):
        not_sent = [output for output in self.outputs if not output.sent]
        for output in not_sent:
            x = abs(curr_pose.x - output.send_pose.x)
            y = abs(curr_pose.y - output.send_pose.y)
            if x <= 3 or y <= 3:  # if x or y is less than 3mm - activate
                print(f"Executing actuator {output._type}")
                output._execute()

    def servo(self, curr_pose:Position=Position()):
        not_moving = [servo for servo in self.servos if not servo.executed]
        moved = 0
        for servo in not_moving:
            x = abs(curr_pose.x - servo.activate_pose.x)
            y = abs(curr_pose.y - servo.activate_pose.y)
            if x <= 3 or y <= 3:  # if x or y is less than 3mm - activate
                print(f"Executing servo {servo._type}")
                servo._execute()

                # Only tracking AX servos, RC servos are sent individually
                if servo.id <= 10:
                    moved += 1

        if moved > 0:
            Servo.send_positions()

if __name__ == "__main__":

    can_handler.start_threads()

    valve = I_O.Valve(0)
    valve._execute()

    time.sleep(1)

    can_handler.stop_threads()







    
