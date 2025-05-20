from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move
from robot_pkg.opponent import Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.misc import *
from robot_pkg.play_elements import *
from robot_pkg.consts import Points
import math


def init_position(init_x, init_y, init_theta, final_position: str, final_rotation: float = 0):
    '''
         The robot is aligned with the left or right corner at the beginning, 
         facing to the right of the area.
         All servos are closed.
         It moves back to the center of area, rotates for 90deg and moves back until it hits a wall.
         Theta is reset after and it moves to the starting position in area.
    '''

    s = Strategy()

    s(task_steps=init_all_servos())

    s(m=Move.ResetOdom(init_x, init_y, init_theta))
    s(m=Move.ResetOdom(init_x, init_y, init_theta))

    if final_position == "left corner":
        s(m=Move.Distance(-122, 100, 100))
    elif final_position == "middle":
        s(m=Move.Distance(-80, 100, 100))
    elif final_position == "right corner":
        s(m=Move.Distance(-42, 100, 100))

    s(m=Move.Rotate(1.5707,  1, 1))

    s(m=Move.Distance(-200, 100, 100))

    angle = init_theta+1.5707
    if angle > math.pi:
        angle -= 2*math.pi
    elif angle < math.pi:
        angle += 2*math.pi

    s(m=Move.ResetOdom(0, 0, angle))
    s(m=Move.ResetOdom(0, 0, angle))

    if final_rotation != 0:
        s(m=Move.Distance(120, 100, 100))
        s(m=Move.RotateTo(final_rotation, 10, 5),
          c=[Condition.CinchPulled(1)])
    else:
        s(m=Move.Distance(150, 100, 100),
          c=[Condition.CinchPulled(1)])

    # s(task_steps=init_all_servos())

    return s.steps


def open_front():
    s = Strategy()

    s(s=[Servo.FrontCenterGrip(FrontCenterGripper.OPEN)])

    return s.steps


def close_front():
    s = Strategy()

    s(s=[Servo.FrontCenterGrip(FrontCenterGripper.CLOSED)])

    return s.steps


def init_all_servos():
    '''
        Brings all servos to their starting position.
        The starting position takes up the least amount of space.
    '''

    s = Strategy()
    s(s=[Servo.FrontCenterGrip(FrontCenterGripper.NEUTRAL),
         Servo.FrontGripLift(FrontGripLift.DOWN),
         Servo.FrontVacuumLift(VacuumLift.UP),
         Servo.FrontVacuum(Vacuum.DOWN),
         Servo.CenterSwing(CenterSwing.INIT),
         Servo.CenterLift(CenterLift.DOWN, 50),
         Servo.BackSwing(BackSwing.HOLD)
         ],
      a=[I_O.Pump(0), I_O.Valve(0), I_O.Magnet(0)])

    return s.steps


def init_front_servos():
    '''
        Brings all front servos to pickup positions.
    '''

    s = Strategy()
    s(s=[Servo.FrontCenterGrip(FrontCenterGripper.OPEN),
         Servo.FrontVacuum(Vacuum.DOWN),
         Servo.CenterSwing(CenterSwing.DOWN, 50),
         Servo.CenterLift(CenterLift.DOWN),
         Servo.FrontGripLift(FrontGripLift.DOWN),
         Servo.FrontVacuumLift(VacuumLift.UP)])

    return s.steps


def init_back_servos():
    '''
         Brings all back servos to their starting position.
    '''

    s = Strategy()
    s(s=[Servo.BackSwing(BackSwing.PICK)])

    return s.steps


def pickup_back_full_stack(back_distance=-200, ID=None):
    '''
       Picks-up and holds one stack with back servos.
       Backing out is not included.
    '''

    s = Strategy()

    # s(c=[Condition.BackSensors(ID)])

    s(m=Move.Distance(back_distance, 1000, 400),
      s=[Servo.BackSwing(BackSwing.PICK)])

    s(s=[Servo.BackSwing(BackSwing.HOLD)])

    return s.steps


def pickup_front_full_stack(forward_distance=250, ID=None):
    '''
        Picks-up and holds one stack with front servos.
        Separating is not included.
        Backing out is not included.
    '''

    s = Strategy()

    s(task_steps=open_front())

    s(m=Move.Distance(forward_distance, 700, 300),  # 300 SA RAZLOGOM
      s=[Servo.FrontVacuum(Vacuum.DOWN),
         Servo.CenterSwing(CenterSwing.DOWN, 50),
         Servo.CenterLift(CenterLift.DOWN),
         Servo.FrontGripLift(FrontGripLift.DOWN),
         Servo.FrontVacuumLift(VacuumLift.HOVER)])

    # s(c=[Condition.FrontSensors(ID)])

    s(s=[Servo.FrontCenterGrip(FrontCenterGripper.GRIP),
         Servo.FrontVacuumLift(VacuumLift.PICKUP2)],
      a=[I_O.Pump(1), I_O.Valve(1), I_O.Magnet(1)])

    s(s=[Servo.FrontGripLift(FrontGripLift.HOVER + 5),   ##### NE RADI SA VISE OD +5 NE ZNAMMMM
         Servo.FrontVacuumLift(VacuumLift.HOVER + 20),
         Servo.CenterLift(CenterLift.HOVER - 15)
         ])

    return s.steps


def two_level():
    '''
        Separates one stack in two mid air. 
        No moving included.
    '''

    s = Strategy()

    s(s=[Servo.FrontVacuumLift(VacuumLift.HOVER + 70)])
    s(s=[Servo.FrontGripLift(100),
         Servo.CenterLift(CenterLift.DOWN),
         Servo.FrontVacuumLift(VacuumLift.HOLD+40)])

    s(s=[Servo.FrontVacuum(Vacuum.MIDDLE),
         Servo.FrontGripLift(FrontGripLift.HOLD),
         Servo.CenterSwing(CenterSwing.UP)])

    s(s=[Servo.CenterLift(CenterLift.HOLD2),
         Servo.FrontGripLift(FrontGripLift.HOVER + 10)])

    s(s=[
        Servo.CenterSwing(CenterSwing.DOWN),
        Servo.FrontVacuum(Vacuum.UP),  # , 30),
        Servo.CenterLift(CenterLift.POSITION2),  # , 50),
        Servo.FrontVacuumLift(VacuumLift.POSITION2+10, 70)])

    return s.steps


def drop_two_level(back_distance=-200):
    '''
       Leaves a two level construction in place and backs out. 
       Backs out.
       Points: 12
    '''

    s = Strategy()

    s(s=[Servo.CenterLift(CenterLift.POSITION2+40),
         Servo.FrontVacuumLift(VacuumLift.POSITION2-20, 50)],
      a=[I_O.Pump(0), I_O.Valve(0)])

    # s(s=[Servo.FrontVacuumLift(VacuumLift.POSITION2-20)])

    s(s=[Servo.FrontVacuum(Vacuum.PUSH),
         Servo.FrontVacuumLift(VacuumLift.DOWN),
         Servo.FrontGripLift(FrontGripLift.DOWN)])

    s(s=[Servo.CenterLift(CenterLift.POSITION2)],
      a=[I_O.Magnet(0)])

    s(m=Move.Distance(back_distance, 1000, 500),
      task_steps=open_front(),
      p=Points.LEVEL1+Points.LEVEL2)

    return s.steps


def drop_one_level(backout_distance=-150, p=0):
    '''
       Leaves a one level construction in place and backs out. 
       Backs out.
       Points: 4
    '''

    s = Strategy()

    s(s=[
        # Servo.CenterLift(CenterLift.DROP1),
        Servo.FrontGripLift(FrontGripLift.DOWN)],
      a=[I_O.Magnet(0)])

    s(m=Move.Distance(backout_distance, 1000, 500),
      p=Points.LEVEL1 + p)

    return s.steps


def drop_separate_two_level(between_drop_distance=-150, backout_distance=-200):
    '''
         Drops lower level and moves back to drop second level.
         Backs out.
         Points: 8
    '''

    s = Strategy()

    s(task_steps=drop_one_level(between_drop_distance))

    s(s=[Servo.CenterLift(CenterLift.DOWN),
         Servo.CenterSwing(CenterSwing.DOWN+8),
         Servo.FrontVacuumLift(VacuumLift.DROP1 + 35),
         Servo.FrontVacuum(Vacuum.DOWN, 45)])

    s(s=[Servo.FrontVacuumLift(VacuumLift.DROP1),
         Servo.CenterSwing(CenterSwing.DOWN)])

    s(s=[Servo.FrontCenterGrip(FrontCenterGripper.OPEN)],
      a=[I_O.Pump(0), I_O.Valve(0)],
      p=Points.LEVEL1)

    s(m=Move.Distance(backout_distance, 1000, 500),
      s=[Servo.FrontVacuumLift(VacuumLift.HOVER)])

    return s.steps


def lift_two_on_one(forward_distance=150, back_distance=-250):
    '''
       Places two levels on a one level high construction on the ground.
       Backs out.
       Points: 25
    '''

    s = Strategy()

    s(s=[Servo.CenterLift(CenterLift.LIFT2-50)],
      a=[I_O.Pump(0), I_O.Valve(0)])

    # s(s=[Servo.CenterLift(CenterLift.UP, 60),
    #      Servo.FrontVacuumLift(VacuumLift.UP, 60),
    #      Servo.FrontGripLift(FrontGripLift.UP-20, 30),
    #      Servo.FrontVacuum(Vacuum.MIDDLE, 60)])
    s(s=[Servo.CenterLift(CenterLift.UP)])

    s(m=Move.Distance(forward_distance, 600, 250),
      s=[Servo.FrontVacuumLift(VacuumLift.UP-15, 60),
         Servo.FrontGripLift(FrontGripLift.UP-20, 50),
         Servo.FrontVacuum(Vacuum.DROP, 60)])

    # s(s=[Servo.FrontVacuumLift(VacuumLift.UP-20)])
    s(s=[Servo.FrontGripLift(FrontGripLift.HOLD),
         Servo.FrontCenterGrip(FrontCenterGripper.OPEN)],
        a=[I_O.Magnet(0)])

    s(m=Move.Distance(back_distance, 1000, 1000),
      p=Points.LEVEL2+Points.LEVEL3)

    return s.steps


def push_two_level(push_distance=150, backout_distance=-300):
    '''
         Pushes a two level construction on top of two planks.
         Makes space for one more two level coonstruction.
         Assumes the robot has backed out after lifting.
         Backs out.
    '''

    s = Strategy()

    # s(s=[Servo.CenterLift(CenterLift.DOWN),
    #      Servo.FrontGripLift(FrontGripLift.DOWN),
    #      Servo.FrontVacuumLift(VacuumLift.PUSH)])

    s(m=Move.Distance(push_distance, 500, 85),
      s=[
        # Servo.CenterLift(CenterLift.DOWN),
        Servo.FrontGripLift(FrontGripLift.DOWN),  # HOVER+20),
        Servo.FrontVacuumLift(10),
        Servo.FrontVacuum(Vacuum.PUSH)])

    s(m=Move.Distance(backout_distance, 1000, 1000))

    return s.steps


def lift_one_on_two(forward_distance=200, backout_distance=-250):
    '''
       Places one level on a two level high construction on the ground.
       Backs out.
       Points: 16
    '''

    s = Strategy()

    s(s=[Servo.CenterLift(CenterLift.UP, 50),
         Servo.CenterSwing(CenterSwing.DOWN+10, 60),
         Servo.FrontVacuumLift(VacuumLift.HOLD, 50)],
      a=[I_O.Pump(0), I_O.Valve(0)])

    s(m=Move.Distance(forward_distance, 1000, 400),  # BILO 500 UBRZANJE
      s=[Servo.FrontVacuumLift(VacuumLift.UP-20),
         Servo.FrontGripLift(FrontGripLift.DOWN),
         Servo.FrontVacuum(Vacuum.DROP, 50)])

    s(s=[Servo.CenterLift(CenterLift.UP-10),
         Servo.CenterSwing(CenterSwing.DOWN),
         Servo.FrontCenterGrip(FrontCenterGripper.OPEN)],
      a=[I_O.Magnet(0)])

    s(m=Move.Distance(backout_distance, 1000, 500), p=Points.LEVEL3)

    return s.steps


def front_lift_stack():
    '''
         Lift entire gripped stack with front servos.
    '''

    s = Strategy()

    s(s=[Servo.FrontVacuum(Vacuum.DOWN),
         Servo.FrontVacuumLift(VacuumLift.UP),
         Servo.FrontGripLift(FrontGripLift.HOLD),
         Servo.CenterLift(CenterLift.HOLD2)])

    return s.steps


def leave_banner(back_distance=-200, forward_distance=125):
    '''
         Leave banner by hitting the back wall.
         Forwards out.
         Points: 20
    '''

    s = Strategy()

    s(m=Move.RotateTo(1.57, 5, 5))

    s(m=Move.Distance(back_distance, 500, 500),
      s=[Servo.BackSwing(BackSwing.PICK)])

    s(m=Move.Distance(forward_distance, 1000, 1000),
      p=Points.BANNER)

    return s.steps


def separate_two_level(rotation):
    '''
        Separate one stack by building two levels, 
        keeping the upper level in center gripper and lower level in back grippers.
    '''

    s = Strategy()

    s(task_steps=drop_one_level(backout_distance=-175, p=-4))
    s(m=Move.RotateTo(rotation, 15, 15),
      s=[Servo.BackSwing(BackSwing.PICK)])
    s(task_steps=pickup_back_full_stack(-255))

    return s.steps


def back_lift_one_on_one():
    '''
       Places one level on a one level high construction on the ground.
    '''

    pass


def back_lift_one_on_stack():
    '''
       Places one level on an untouched stack (with four cans and two planks) 
       on the ground.
       Pushes the untouched stack into an area.
    '''

    pass


def drop_back_one_level(rotation=0, forward_distance=250, p=0):
    '''
       Drop one level with back servos.
       Forwards out.
       Points: 4
    '''

    s = Strategy()

    # if (rotation != 0):
    s(m=Move.RotateTo(rotation, 10, 5))
    # else:
    #     s(s=[Servo.BackLift(BackGripLift.DOWN)])

    s(s=[Servo.BackSwing(BackSwing.DROP)])

    s(m=Move.Distance(forward_distance, 500, 500),
        p=Points.LEVEL1 + p)

    return s.steps


def drop_back_two_level():
    '''
       Drop two level construction with back servos.
       Forwards out.
       Points: 8
    '''

    pass


def sima_coordinates(sima1_coor: list[Position] = [], sima2_coor: list[Position] = [], sima3_coor: list[Position] = [], sima4_coor: list[Position] = []):
    '''
         Lists coordinates that will be sent to SIMAs at the 85th second of the match.
         Each position is defined with (x, y, theta, speed).
         The firt coordinate defines the starting position and orientation of the SIMA, and
         the rest the target coordinates on it's path.
    '''
    s = Strategy()

    s(sima_id=1,
      sima=sima1_coor)

    s(sima_id=2,
      sima=sima2_coor)

    s(sima_id=3,
      sima=sima3_coor)

    s(sima_id=4,
      sima=sima4_coor)

    return s.steps
