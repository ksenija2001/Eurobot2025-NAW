from threading import Thread
import time
from robot_pkg.strategy import Strategy 
from robot_pkg.servo import Servo
from robot_pkg.move import Move
from robot_pkg.io import I_O, SensorType
from robot_pkg.data import Variables
# from lidar import Lidar
from multiprocessing import Event
from robot_pkg.conditions import ConditionType 

class Execute:

    def __init__(self, strategy:Strategy): 
        self.steps = strategy.steps
        self.thread = Thread(target=self.loop, args=())

        # self.nucleo = Nucleo(debug=False)
        # self.nucleo.start()
        # self.actuators = Actuators()
        # self.sensors = Sensors()
        # self.sensors.start()
        # self.servo_moving = ServoMoving()
        # self.servo_moving.start()
        # self.display = Display()
        # self.front_detection = Event()
        # self.back_detection = Event()
        # self.lidar = Lidar(self.nucleo, self.front_detection, self.back_detection)
        # self.lidar.start()

        self.is_active = False
        
    def start(self):
        self.is_active = True
        self.thread.start()

    def loop(self):
        next_step_id = None

        while self.is_active:
           
           # Ako step ima ID to znaci da skaceu potpuno novu granu strategije
           # Step na koji necemo da skacemo ima ID = None
           # Step na koji hocemo da skocimo ima ID, i ako je jednak zeljenom next step id izvrsice njega
            step = self.steps.pop(0)
            while step.ID != next_step_id:
                step = self.steps.pop(0)
            
            print(step.conditions)

            start_time = time.time()
            step.step()

            # time.sleep(0.7)
            # Waiting for end of step
            while self.is_active:
                cinch = I_O.sensor_states[SensorType.CINCH.value].is_set()
                move_done = Move.move_done.is_set()
                servo_in_pos = Servo.check_in_positions()
                # print(f"Servo: {servo_in_pos}")

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
                time.sleep(0.1)
                print("-------------------------------")

                if len(self.steps) == 0:
                    self.is_active = False
                    
                break
            
    def stop(self):
        # self.nucleo.set_motor_speed(0,0,5000)
        # time.sleep(0.1)
        # self.nucleo.stop()
        # self.actuators.stop()
        # self.lidar.stop()
        # self.sensors.stop()
        # self.servo_moving.stop()
        
        self.is_active = False
        self.thread.join()
