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
            print(step.conditions)

            start_time = time.time()

            # rc_servo_id = None
            # for cond_tuple in step.conditions:
            #     if ConditionType.SERVO_POSITION in cond_tuple:  
            #         rc_servo_id = cond_tuple[2]
            #         rc_servo_position = cond_tuple[3]
            #         Servo.check_position(rc_servo_id)
            #         time.sleep(0.1)
            #         if rc_servo_position*0.95 < Servo.servo_positions[rc_servo_id] <= rc_servo_position*1.05:
            #             break
            #         else:
            #             next_step_id = cond_tuple[1]

            # Activate servos and send outputs based on current position
            step.move()            # starts movement
            step.servo()  # activates servos that do not have a specified pose
            step.output() # sends outputs that do not have a specified pose

            # Waiting for end of step and checking conditions
            while self.running:
                cinch = I_O.sensor_states[SensorType.CINCH.value].is_set()
                move_done = Move.move_done.is_set()
                curr_pose = Move.pose
                servo_in_pos = Servo.check_in_positions()
                # print(f"Servo: {servo_in_pos}")

                 # Activate servos and send outputs based on current position
                step.move()            # starts movement
                step.servo(curr_pose)  # activates servos that do not have a specified pose
                step.output(curr_pose) # sends outputs that do not have a specified pose


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

                args = [start_time, time.time(), move_done, cinch, servo_in_pos] 
                checked = {cond.type : cond.check(args) for cond in step.conditions}
              
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
