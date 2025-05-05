from threading import Thread, Event
import time, struct
from robot_pkg.display import Display
from robot_pkg.step import Step
from robot_pkg.strategy import Strategy 
from robot_pkg.servo import Servo
from robot_pkg.move import Move, MoveType
from robot_pkg.in_out import I_O, SensorType
from robot_pkg.consts import Variables
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.main import log_handler
from robot_pkg.sima_communication import SIMA
from robot_pkg.misc import FrontCenterLeft, FrontCenterRight, FrontSideLeft, FrontSideRight, \
                            BackCenterLeft, BackCenterRight, BackSideLeft, BackSideRight


class Execute:

    def __init__(self, strategy:Strategy, main_running:Event): 
        self.steps = strategy.steps
        self.thread = Thread(target=self.loop, args=())
        self._logger = log_handler.get_logger("execute")
        self.running = False
        self.main_running = main_running
        self.display = Display()

        self.sima = SIMA()
        
    def start(self):
        self.running = True

        self.display.start()

        # Reads sima coordinates before starting communication
        for i in range(4):
            if self.steps[i].sima_id is not None:
                sima_id = self.steps[i].sima_id
                coor = self.steps[i].sima
                if len(coor) > 0:
                    print(f"SIMA COORDINATES FOR {sima_id}")
                    self.sima.coordinates[sima_id] = coor

        self.sima.start_threads()

        self.thread.start()


    def loop(self):
        next_step_id = None
        last_moving_step = None

        while self.running:
           # next_step_id will be None while the strategy is executing linearly
           # when next_step_id is an integer, all steps with an ID not equal to next_step_ID will be skipped

            step = self.steps.pop(0)
            while step.ID != next_step_id: # and step.ID != 100:
                step = self.steps.pop(0)
            
            self._logger.info(f"Current step ID: {step.ID}")
            self._logger.info(f"Conditions: {step.conditions}")

            start_time = time.time()

            # Conditions that need to be checked before start of step
            to_break = False
            for cond in step.conditions: 
                if cond._type == ConditionType.BACK:
                    back_sensor_state = I_O.sensor_states[SensorType.BACK.value] 
                    next_step_id = cond.check([None, None, None, None, None, None, back_sensor_state])
                    if next_step_id != False:
                        to_break = True
                        self._logger.info(f"Condition met TYPE: {ConditionType.BACK}")

                    sensor = [cond for cond in step.conditions if cond._type == ConditionType.BACK][0]
                    step.conditions.remove(sensor)
                    break

                if cond._type == ConditionType.FRONT:
                    center_front = I_O.sensor_states[SensorType.FRONT_CENTER_LEFT.value] or I_O.sensor_states[SensorType.FRONT_CENTER_RIGHT.value]
                    side_front = I_O.sensor_states[SensorType.FRONT_LEFT.value] or I_O.sensor_states[SensorType.FRONT_RIGHT.value] 
                    front_sensor_state = center_front and side_front  # At least one side and one center, else there is probably no plank
                    
                    next_step_id = cond.check([None, None, None, None, None, front_sensor_state, None])
                    if next_step_id != False:
                        to_break = True
                        self._logger.info(f"Condition met TYPE: {ConditionType.FRONT}")
                        
                    sensor = [cond for cond in step.conditions if cond._type == ConditionType.FRONT][0]
                    step.conditions.remove(sensor)
                    break
        
            if to_break:
                continue

            # Activate servos and send outputs that do not depend on current position
            if step.movement is not None:
                last_moving_step = step
                step.move()    
            step.servo()  
            step.output() 

            # time.sleep(0.5)
            # Waiting for end of step and checking conditions
            while self.running:
                time.sleep(0.001)

                cinch = I_O.sensor_states[SensorType.CINCH.value]
                move_done = Move.move_done.is_set()
                curr_pose = Move.pose
                servo_in_pos = Servo.check_in_positions()

                # Activate servos and send outputs based on current position
                step.servo(curr_pose)  
                step.output(curr_pose) 

                # TODO check if it will always enter this condition
                # front or back detection wouldn't be enabled if the robot wasn't moving forward or backward
                if (Variables.front_detection.is_set() or Variables.back_detection.is_set()): 
                    self._logger.info("\n*********\nDETECTION\n*********\n")

                    Variables.front_detection.clear()
                    Variables.back_detection.clear()

                    # if there was a detection condition and the attempts ran out in current step a skip to a new step happens
                    detection_cond = [c for c in step.conditions if c._type == ConditionType.DETECTION] 
                    
                    if len(detection_cond) > 0 and detection_cond[0].attempts == 0:
                        next_step_id = detection_cond[0].ID
                        break
                    elif len(detection_cond) > 0 and detection_cond[0].attempts > 0:                            
                        detection_cond[0].attempts -= 1

                    # if a condition wasn't set, it will attemp indefinetly
                    self.steps.insert(0, Step(None, Move.Detection(100), [], [], [Condition.InPosition(None), Condition.MatchTime(100, 96)], None, None, 0))
                                       
                    if step.movement is None:
                        step.movement = last_moving_step.movement

                    step.movement.executed = False

                    if step.movement._type == MoveType.SPLINE.name:
                        data = bytearray(step.movement.data)
                        data[2:6] = bytearray(struct.pack('f', 300))
                        print(f"SENDING: {data}")
                        step.movement.data = bytes(data)

                    Variables.processing_detection.set()

                    self.steps.insert(1, step)

                    break

                args = [start_time, time.time(), move_done, cinch, servo_in_pos, None, None] 
                checked = {cond._type : cond.check(args) for cond in step.conditions}
              
                if len(step.conditions) == 0 or \
                    (len(step.conditions) == 1 and step.conditions[0]._type == ConditionType.SIMA):
                    # MOVEMENT ADDED BEFORE TASK
                    #pass
                    next_step_id = None
                    break
                # Conditions that are continouosly checked during step execution
                elif ConditionType.SIMA in checked and checked[ConditionType.SIMA] != False:
                    self._logger.info(f"Condition met TYPE: {ConditionType.SIMA}")

                    sima = [cond for cond in step.conditions if cond._type == ConditionType.SIMA][0]
                    step.conditions.remove(sima)
                    self.sima.send_start()

                    continue
                elif ConditionType.TIME in checked and checked[ConditionType.TIME] != False: 
                    self._logger.info(f"Condition met TYPE: {ConditionType.TIME}")
                    next_step_id = checked[ConditionType.TIME]  
                elif ConditionType.TIMEOUT in checked and checked[ConditionType.TIMEOUT] != False:
                    self._logger.info(f"Condition met TYPE: {ConditionType.TIMEOUT}")
                    next_step_id = checked[ConditionType.TIMEOUT]
                    # if not Servo.check_in_positions():
                    #     # TODO check if it's a problem if not all servos are in position
                    #     continue
                elif ConditionType.CINCH in checked and checked[ConditionType.CINCH] != False:
                    self._logger.info(f"Condition met TYPE: {ConditionType.CINCH}")
                    next_step_id = checked[ConditionType.CINCH]
                elif ConditionType.POSITION in checked and ConditionType.SERVO in checked:
                    if checked[ConditionType.POSITION] != False and checked[ConditionType.SERVO] != False:
                        self._logger.info(f"Condition met TYPE: {ConditionType.POSITION} and {ConditionType.SERVO}")
                        next_step_id = checked[ConditionType.POSITION]
                    else:
                        continue
                elif ConditionType.POSITION in checked and checked[ConditionType.POSITION] != False:
                    self._logger.info(f"Condition met TYPE: {ConditionType.POSITION}")
                    next_step_id = checked[ConditionType.POSITION]
                elif ConditionType.SERVO in checked and checked[ConditionType.SERVO] != False:
                    self._logger.info(f"Condition met TYPE: {ConditionType.SERVO}")
                    next_step_id = checked[ConditionType.SERVO]
                else:
                    continue
                
                Variables.points += step.points
                self.display.add_points(step.points)       

                time.sleep(0.05)
                self._logger.info("-------------------------------")

                if len(self.steps) == 0:
                    self.running = False
                    
                break
        
        self.main_running.clear()
            
    def stop(self):
        self.running = False
        self.thread.join()

        self.main_running.clear()

        stop_motors = Move.Stop()
        stop_motors._execute()

        self.sima.stop_threads()


        actuators = [I_O.Pump(0), I_O.Valve(0)]
        for actuator in actuators:
            actuator._execute()
            time.sleep(0.01)
        
        grippers = [Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN), 
                    Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN), 
                    Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN),
                    Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN)]
        
        for gripper in grippers:
            if type(gripper) is tuple:
                gripper[0]._execute()
                time.sleep(0.01)
                gripper[1]._execute()
            else:
                gripper._execute()
            time.sleep(0.01)



