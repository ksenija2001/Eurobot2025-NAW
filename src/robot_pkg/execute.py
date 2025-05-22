from threading import Thread, Event
import time
import struct
import math
from robot_pkg.display import Display
from robot_pkg.step import Step
from robot_pkg.strategy import Strategy
from robot_pkg.servo import Servo
from robot_pkg.move import Move, MoveType, Position
from robot_pkg.in_out import I_O, SensorType
from robot_pkg.consts import Variables
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.main import log_handler
from robot_pkg.sima_communication import SIMA
from robot_pkg.play_elements import MaterialStack
# from robot_pkg.misc import FrontCenterLeft, FrontCenterRight


class Execute:

    def __init__(self, strategy: Strategy, main_running: Event):
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
                    self.sima.coordinates[sima_id] = coor

        self.sima.start_threads()

        self.thread.start()

    def loop(self):
        next_step_id = None
        last_moving_step = None
        step = None
        reached_home_step = False
        start_pose = None

        while self.running:
           # next_step_id will be None while the strategy is executing linearly
           # when next_step_id is an integer, all steps with an ID not equal to next_step_ID will be skipped

            try:
                step = self.steps.pop(0)
                while step.ID != next_step_id:
                    step = self.steps.pop(0)

            except IndexError:
                self._logger.info("END OF STRATEGY")
                self.running = False
                break

            self._logger.info(f"Current step ID: {step.ID}")
            self._logger.info(f"Conditions: {step.conditions}")

            if step.movement is None and \
                    len(step.servos) == 0 and \
                    len(step.outputs) == 0 and \
                    len(step.conditions) < 1:

                Variables.points += step.points
                self.display.add_points(step.points)
                next_step_id = None
                continue

            if step.ID == 100:
                reached_home_step = True

                data = bytearray(step.movement.data)
                distance, v, a = struct.unpack('3f', data)

                direction = -1 if Move.pose.theta < 0 else 1
                step.movement.data = struct.pack(
                    '3f', direction*distance, v, a)

            start_time = time.time()

            # Conditions that need to be checked before start of step
            to_break = False
            for cond in step.conditions:
                if cond._type == ConditionType.STACK:
                    to_break = True

                    stacks = [name for name, value in MaterialStack.get_all_unvisited_stacks()]
                    
                    stack_present = cond.stack in stacks

                    print(f"{cond.stack}: {stack_present}")

                    next_step_id = cond.check(
                        [None, None, None, None, None, None, None, stack_present])
                    
                    print(f"Next step ID: {next_step_id}")

                    if next_step_id != False:
                        self._logger.info(
                            f"Condition met TYPE: {ConditionType.STACK}")
                        break
                    else:
                        next_step_id = None
                        
                    # sensor = [
                    #     cond for cond in step.conditions if cond._type == ConditionType.BACK][0]
                    # step.conditions.remove(sensor)
                    # break
                if cond._type == ConditionType.BACK:
                    to_break = True


                    back_sensor_state = I_O.sensor_states[SensorType.BACK.value]
                    
                    print(f"BACK SENSORS: {back_sensor_state}")

                    next_step_id = cond.check(
                        [None, None, None, None, None, None, back_sensor_state, None])
                    
                    print(f"Next step ID: {next_step_id}")

                    if next_step_id != False:
                        self._logger.info(
                            f"Condition met TYPE: {ConditionType.BACK}")
                        break
                    else:
                        next_step_id = None
                        
                    # sensor = [
                    #     cond for cond in step.conditions if cond._type == ConditionType.BACK][0]
                    # step.conditions.remove(sensor)
                    break

                if cond._type == ConditionType.FRONT:
                    to_break = True

                    center_front = I_O.sensor_states[SensorType.FRONT_CENTER_LEFT.value] or I_O.sensor_states[
                        SensorType.FRONT_CENTER_RIGHT.value]
                    side_front = I_O.sensor_states[SensorType.FRONT_LEFT.value] or I_O.sensor_states[SensorType.FRONT_RIGHT.value]
                    # At least one side and one center, else there is probably no plank
                    front_sensor_state = center_front and side_front

                    print(f"FRONT SENSORS: {front_sensor_state}")

                    next_step_id = cond.check(
                        [None, None, None, None, None, front_sensor_state, None, None])

                    print(f"Next step ID: {next_step_id}")
                    if next_step_id != False:
                        self._logger.info(
                            f"Condition met TYPE: {ConditionType.FRONT}")
                        break
                    else:
                        next_step_id = None

                    # sensor = [
                    #     cond for cond in step.conditions if cond._type == ConditionType.FRONT][0]
                    # step.conditions.remove(sensor)
                    break

            if to_break:
                continue

            # Activate servos and send outputs that do not depend on current position
            if step.movement is not None:
                start_pose = Position(Move.pose.x, Move.pose.y, Move.pose.theta, 0)
                last_moving_step = step
                step.move()
            step.servo()
            step.output()

            # Waiting for end of step and checking conditions
            while self.running:
                time.sleep(0.01)

                cinch = I_O.sensor_states[SensorType.CINCH.value]
                move_done = Move.move_done.is_set()
                curr_pose = Move.pose
                servo_in_pos = Servo.check_in_positions()

                # Activate servos and send outputs based on current position
                step.servo(curr_pose)
                step.output(curr_pose)

                # front or back detection wouldn't be enabled if the robot wasn't moving forward or backward
                if (Variables.front_detection.is_set() or Variables.back_detection.is_set()):
                    self._logger.info("\n*********\nDETECTION\n*********\n")

                    Variables.front_detection.clear()
                    Variables.back_detection.clear()

                    # if there was a detection condition and the attempts ran out in current step a skip to a new step happens
                    detection_cond = [
                        c for c in last_moving_step.conditions if c._type == ConditionType.DETECTION]

                    print(f"Detection condition: {detection_cond}")
                    if len(detection_cond) > 0 and detection_cond[0].attempts == 0:
                        # next_step_id = detection_cond[0].ID
                        self.steps.insert(0, Step(None, Move.Detection(100), [], [], [
                                        Condition.InPosition(detection_cond[0].ID), Condition.MatchTime(100, 96)], None, None, 0))

                        break
                    elif len(detection_cond) > 0 and detection_cond[0].attempts > 0:
                        print(f"Detection attempts: {detection_cond[0].attempts}")

                        last_moving_step.conditions.remove(detection_cond[0])
                        detection_cond[0].attempts -= 1
                        last_moving_step.conditions.append(detection_cond[0])

                   # if a condition wasn't set, it will attemp indefinetly
                    self.steps.insert(0, Step(None, Move.Detection(100), [], [], [
                                      Condition.InPosition(next_step_id), Condition.MatchTime(100, 96)], None, None, 0))


                    step = last_moving_step
                    next_step_id = None
                                        
                    position_cond = [cond for cond in step.conditions if cond._type == ConditionType.POSITION]
                    if len(position_cond) == 0:
                        step.conditions.extend([Condition.InPosition(None), Condition.MatchTime(100, 96)])
                    step.movement.executed = False

                    if step.movement._type == MoveType.SPLINE.name:
                        data = bytearray(step.movement.data)
                        size = data[0]
                        direction = chr(data[1])
                        index = 1+1+4+4*3*(size-1)
                        x, y, theta = struct.unpack(
                            '3f', data[index:index+12])

                        reverse = math.pi if direction == 'r' else 0
                        x_ = x + 200*math.cos(theta+math.pi+reverse)
                        y_ = y + 200*math.sin(theta+math.pi+reverse)

                        move1 = Move.To(x_, y_, direction, 1000, 500, 5, 5)
                        step.movement = move1

                        move2 = Move.To(x, y, direction, 1000, 500, 5, 5)
                        step2 = Step(None, move2, [], [], [Condition.InPosition(
                            None), Condition.MatchTime(100, 96)], None, None, 0)

                        self.steps.insert(1, step2)
                    elif step.movement._type == MoveType.DISTANCE.name:
                        data = bytearray(step.movement.data)
                        p, v, a = struct.unpack('3f', data)

                        target_pose_x = start_pose.x + math.cos(start_pose.theta) * p
                        target_pose_y = start_pose.y + math.sin(start_pose.theta) * p

                        direction = 'f' if p > 0 else 'r'
                        
                        move = Move.To(target_pose_x, target_pose_y, direction, v, a, 10, 5)
                        # distance_from_start = math.sqrt((curr_pose.x - start_pose.x)**2 + (curr_pose.y - start_pose.y)**2)

                        # p += 100 * abs(p)/p

                        # new_p = p - distance_from_start * abs(p)/p + 100 * abs(p)/p

                        # move = Move.Distance(new_p, v, a)
                        step.movement = move

                    Variables.processing_detection.set()
                    # Move.move_done.clear()
                    self.steps.insert(1, step)

                    break

                args = [start_time, time.time(), move_done, cinch,
                        servo_in_pos, None, None]
                checked = {cond._type: cond.check(
                    args) for cond in step.conditions}

                # Send start to SIMAs in 85th second
                if time.time() - Variables.match_start_time >= 84.5 and not self.sima.sent:
                    self.sima.send_start()

                # Conditions that are continouosly checked during step execution
                if len(step.conditions) == 0 or \
                    (len(step.conditions) == 1 and step.conditions[0]._type == ConditionType.DETECTION):
                    next_step_id = None
                    
                elif ConditionType.TIME in checked and checked[ConditionType.TIME] != False and \
                    not (checked[ConditionType.TIME] == 100 and reached_home_step):
                    self._logger.info(
                        f"Condition met TYPE: {ConditionType.TIME}")
                    next_step_id = checked[ConditionType.TIME]
                elif ConditionType.TIMEOUT in checked and checked[ConditionType.TIMEOUT] != False:
                    self._logger.info(
                        f"Condition met TYPE: {ConditionType.TIMEOUT}")
                    next_step_id = checked[ConditionType.TIMEOUT]

                elif ConditionType.CINCH in checked and checked[ConditionType.CINCH] != False:
                    self._logger.info(
                        f"Condition met TYPE: {ConditionType.CINCH}")
                    next_step_id = checked[ConditionType.CINCH]
                elif ConditionType.POSITION in checked and ConditionType.SERVO in checked:
                    if checked[ConditionType.POSITION] != False and checked[ConditionType.SERVO] != False:
                        self._logger.info(
                            f"Condition met TYPE: {ConditionType.POSITION} and {ConditionType.SERVO}")
                        next_step_id = checked[ConditionType.POSITION]
                    else:
                        continue
                elif ConditionType.POSITION in checked and checked[ConditionType.POSITION] != False:
                    self._logger.info(
                        f"Condition met TYPE: {ConditionType.POSITION}")
                    next_step_id = checked[ConditionType.POSITION]
                elif ConditionType.SERVO in checked and checked[ConditionType.SERVO] != False:
                    self._logger.info(
                        f"Condition met TYPE: {ConditionType.SERVO}")
                    next_step_id = checked[ConditionType.SERVO]
                else:
                    continue

                Variables.points += step.points
                self.display.add_points(step.points)

                time.sleep(0.01) # 0.025
                self._logger.info("-------------------------------")

                # if len(self.steps) == 0:
                #     self.running = False

                break

        self.main_running.clear()

    def stop(self):
        self.running = False
        time.sleep(0.1)

        self.stop_children()

        self.thread.join()

    def stop_children(self):
        stop_motors = Move.Stop()
        stop_motors._execute()

        log_handler.get_logger("move").info(f"Executing movement STOP")

        self.sima.stop_threads()

        actuators = [I_O.Pump(0), I_O.Valve(0), I_O.Magnet(0)]

        for actuator in actuators:
            actuator._execute()
            time.sleep(0.01)

        # grippers = [Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
        #             Servo.FrontSideGrip(FrontSideLeft.OPEN,
        #                                 FrontSideRight.OPEN),
        #             Servo.BackCenterGrip(
        #                 BackCenterLeft.OPEN, BackCenterRight.OPEN),
        #             Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN)]

        # for gripper in grippers:
        #     if type(gripper) is tuple:
        #         gripper[0]._execute()
        #         time.sleep(0.01)
        #         gripper[1]._execute()
        #     else:
        #         gripper._execute()
        #     time.sleep(0.01)
