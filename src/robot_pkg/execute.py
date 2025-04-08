from threading import Thread
import time
from robot_pkg.strategy import Strategy 
from robot_pkg.servo import Servo
from robot_pkg.move import Move, Position
from robot_pkg.in_out import I_O, SensorType
from robot_pkg.data import Variables
from robot_pkg.conditions import ConditionType 

class Execute:

    def __init__(self, strategy:Strategy): 
        self.steps = strategy.steps
        self.thread = Thread(target=self.loop, args=())
        self.running = False
        
    def start(self):
        self.running = True
        self.thread.start()

    def loop(self):
        next_step_id = None

        while self.running:
           # next_step_id will be None while the strategy is executing linearly
           # when next_step_id is an integer, all steps with an ID not equal to next_step_ID will be skipped

            step = self.steps.pop(0)
            while step.ID != next_step_id:
                step = self.steps.pop(0)
            
            print(f"Current step ID: {step.ID}")
            print(f"Conditions: {step.conditions}")

            start_time = time.time()

            # Conditions that need to be checked before start of step
            for cond in step.conditions:
                if cond._type == ConditionType.SERVO_POSITION:  
                    position = Servo.check_position(cond.servo_id)
                    time.sleep(0.1)
                    next_step_id = cond.check([None, None, None, None, None, position])
                    if next_step_id != False:
                        continue
                         
            # Activate servos and send outputs that do not depend on current position
            step.move()    
            step.servo()  
            step.output() 

            # Waiting for end of step and checking conditions
            while self.running:
                cinch = I_O.sensor_states[SensorType.CINCH.value].is_set()
                move_done = Move.move_done.is_set()
                curr_pose = Move.pose
                servo_in_pos = Servo.check_in_positions()

                # Activate servos and send outputs based on current position
                step.servo(curr_pose)  
                step.output(curr_pose) 

                # if (self.front_detection.is_set() or self.back_detection.is_set()) and \
                #     step.movement is not None and step.movement.type == MoveType.TO_XY and \
                #     not cinch:

                #     print("DETECTION")
                #     self.nucleo.set_motor_speed(0, 0, 2000)
                #     time.sleep(1)
                #     if self.front_detection.is_set():
                #         dist = -200
                #     else: # back detection
                #         dist = 200
                #     self.nucleo.move_distance(dist)
                #     time.sleep(2)
                #     self.steps.insert(step, 0)
                #     break

                args = [start_time, time.time(), move_done, cinch, servo_in_pos, None] 
                checked = {cond._type : cond.check(args) for cond in step.conditions}
              
                # Conditions that are continouosly checked during step execution
                if ConditionType.TIME in checked and checked[ConditionType.TIME] != False: 
                    print(f"Condition met TYPE: {ConditionType.TIME}")
                    next_step_id = checked[ConditionType.TIME]  
                elif ConditionType.TIMEOUT in checked and checked[ConditionType.TIMEOUT] != False:
                    print(f"Condition met TYPE: {ConditionType.TIMEOUT}")
                    next_step_id = checked[ConditionType.TIMEOUT]
                    # if not Servo.check_in_positions():
                    #     # TODO check if it's a problem if not all servos are in position
                    #     continue
                elif ConditionType.CINCH in checked and checked[ConditionType.CINCH] != False:
                    print(f"Condition met TYPE: {ConditionType.CINCH}")
                    next_step_id = checked[ConditionType.CINCH]
                elif ConditionType.POSITION in checked and ConditionType.SERVO in checked:
                    if checked[ConditionType.POSITION] != False and checked[ConditionType.SERVO] != False:
                        print(f"Condition met TYPE: {ConditionType.POSITION} and {ConditionType.SERVO}")
                        next_step_id = checked[ConditionType.POSITION]
                    else:
                        continue
                elif ConditionType.POSITION in checked and checked[ConditionType.POSITION] != False:
                    print(f"Condition met TYPE: {ConditionType.POSITION}")
                    next_step_id = checked[ConditionType.POSITION]
                elif ConditionType.SERVO in checked and checked[ConditionType.SERVO] != False:
                    print(f"Condition met TYPE: {ConditionType.SERVO}")
                    next_step_id = checked[ConditionType.SERVO]
                elif len(step.conditions) == 0:
                    pass
                else:
                    continue
                
                Variables.points += step.points
                #self.display.setNumber(Variables.points)
                # self.nucleo.set_motor_speed(0, 0, 2000)
                # time.sleep(0.1)
                print("-------------------------------")

                if len(self.steps) == 0:
                    self.running = False
                    
                break
            
    def stop(self):
        # self.nucleo.set_motor_speed(0,0,5000)
        # time.sleep(0.1)
        # self.nucleo.stop()
        # self.actuators.stop()
        # self.lidar.stop()
        # self.sensors.stop()
        # self.servo_moving.stop()
        
        self.running = False
        self.thread.join()
