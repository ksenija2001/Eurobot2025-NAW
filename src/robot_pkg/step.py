import time

from robot_pkg.main import log_handler
from robot_pkg.conditions import Condition
from robot_pkg.servo import Servo
from robot_pkg.move import Move, Position
from robot_pkg.in_out import I_O

class Step:
    def __init__(self, ID, movement:Move, outputs:list[I_O], servos:list[Servo], conditions:list[Condition], sima_id:int, sima_coor:list[Position], points):
        self.ID = ID
        self.movement = movement
        self.outputs = outputs
        self.servos = servos
        self.conditions = [cond for cond in conditions]  # needs to be copied because the list is cleared in step afterward
        self.sima_id = sima_id
        self.sima = sima_coor
        self.points = points

    def move(self):
        if self.movement is not None and not self.movement.executed:
            log_handler.get_logger("move").info(f"Executing movement {self.movement._type}")
            self.movement._execute()
            
    def output(self, curr_pose:Position=Position()):
        not_sent = [output for output in self.outputs if not output.sent]
        for output in not_sent:
            x = abs(curr_pose.x - output.send_pose.x)
            y = abs(curr_pose.y - output.send_pose.y)
            if x <= 3 or y <= 3:  # if x or y is less than 3mm - activate
                print(f"Executing actuator {output._type}")
                output._execute()
                time.sleep(0.01)
        
        if len(not_sent) > 0:
            time.sleep(0.2)

    def servo(self, curr_pose:Position=Position()):
        not_moving = [servo for servo in self.servos if not servo.executed]
        moved = 0
        for servo in not_moving:
            x = abs(curr_pose.x - servo.activate_pose.x)
            y = abs(curr_pose.y - servo.activate_pose.y)
            if x <= 50 or y <= 50:  # if x or y is less than 3mm - activate
                print(f"Executing servo {servo._type}")
                servo._execute()

                # Only tracking AX servos, RC servos are sent individually
                if servo.id <= 10:
                    moved += 1

                time.sleep(0.01)

        if moved > 0:
            Servo.send_positions()

if __name__ == "__main__":
    pass

    # can_handler.start_threads()

    # valve = I_O.Valve(0)
    # valve._execute()

    # time.sleep(1)

    # can_handler.stop_threads()







    
