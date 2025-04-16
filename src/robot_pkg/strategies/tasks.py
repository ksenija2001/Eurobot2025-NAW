from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType
from robot_pkg.misc import *

def init_front_servos():
    '''
        Brings all front servos to their starting position
    '''
    s = Strategy()
    s(s=[Servo.FrontSideGrip(FRONT_SIDE_GRIP_OPEN, 100),
         Servo.FrontCenterGrip(FRONT_CENTER_GRIP_OPEN, 100),
         Servo.FrontGripLift(FRONT_GRIP_LIFT_DOWN, 100),
         Servo.FrontVacuumLift(FRONT_VACUUM_LIFT_UP, 100),
         Servo.FrontVacuum(FRONT_VACUUM_DOWNWARD, 100),
         Servo.CenterSwing(CENTER_SWING_DOWN, 100),
         Servo.CenterLift(CENTER_LIFT_DOWN, 100)])

    return s.steps

def init_back_servos():
    '''
        Brings all back servos to their starting position
    '''
    s = Strategy()
    s(s=[Servo.BackSideGrip(BACK_SIDE_GRIP_OPEN, 100),
         Servo.BackCenterGrip(BACK_CENTER_GRIP_OPEN, 100),
         Servo.BackLift(BACK_LIFT_BANNER, 100)])

    return s.steps

def pickup_front_regular(x, y, yaw, offset_x, offset_y, offset_yaw):
    '''
        Given a location 15 cm from the stack, pickus up the stack with front grippers.
    '''
    s = Strategy()

    s(m=Move.To(x + offset_x, y + offset_y, yaw + offset_yaw, 100, 500, FRONT))

    s(m=Move.Distance(150, 100, 300))

    s(s=[Servo.FrontSideGrip(FRONT_SIDE_GRIP_CLOSED, 100),
         Servo.FrontCenterGrip(FRONT_CENTER_GRIP_CLOSED, 100),
         Servo.FrontVacuumLift(FRONT_VACUUM_LIFT_PICKUP, 100)]) # puts the vaccums on top of the planks to hold them while carrying
    
    # consider also lifting the cans a bit while carrying them
    return s.steps

def pickup_back_regular(x, y, yaw, offset_x, offset_y, offset_yaw):
    '''
        Given a location 15 cm from the stack, pickus up the stack with back grippers.
    '''
    s = Strategy()

    s(m=Move.To(x + offset_x, y + offset_y, yaw + offset_yaw, 100, 500, BACK))

    s(m=Move.Distance(150, 100, 300))

    s(s=[Servo.BackSideGrip(BACK_SIDE_GRIP_CLOSED, 100),
         Servo.BackCenterGrip(BACK_CENTER_GRIP_CLOSED, 100)])

    return s.steps

def pickup_front_two_level(x, y, yaw, offset_x, offset_y, offset_yaw):
    '''
        Given a location 15 cm from the stack, picks up a second stack with the front side, putting the first on top of it.
        Rarely used, but you never know.
    '''
    s = Strategy()

    s(m=Move.To(x + offset_x, y + offset_y, yaw + offset_yaw, 100, 500, FRONT))
    s(s=[Servo.FrontVacuumLift(FRONT_VACUUM_LIFT_UP, 100), 
         Servo.FrontVacuum(FRONT_VACUUM_OUTSTRETCHED, 100),
         Servo.FrontGripLift(FRONT_GRIP_LIFT_UP, 20)]) # slowly lifting the stack so the planks wouldn't fall

     # move to the stack with the first stack lifted above the targeted one
    s(m=Move.Distance(150, 100, 300))

     # open grippers, dropping them on the planks of the lower stack
    s(s=[Servo.FrontCenterGrip(FRONT_CENTER_GRIP_OPEN, 100),
      Servo.FrontSideGrip(FRONT_SIDE_GRIP_OPEN, 100)])
    
    # a little rikverc
    s(m=Move.Distance(-150, 100, 300))

     # lower the grippers for picking up the two-level stack, while moving towards it
    s(s=Servo.FrontGripLift(FRONT_GRIP_LIFT_DOWN, 100),
      m=Move.Distance(150, 100, 300))
    
    # close the grippers and put the vacuum grippers on top to hold
    s(s=[Servo.FrontSideGrip(FRONT_SIDE_GRIP_CLOSED, 100),
          Servo.FrontCenterGrip(FRONT_CENTER_GRIP_CLOSED, 100),
      Servo.FrontVacuumLift(FRONT_VACUUM_LIFT_UP, 100)]) #
    

    


    return s.steps

def two_level():
    '''
        Build two levels from one material stock.
    '''
    s = Strategy()

    s(m=Move.Distance(150, 100, 300))

    s(s=[Servo.FrontSideGrip(FRONT_SIDE_GRIP_CLOSED, 100),
         Servo.FrontCenterGrip(FRONT_CENTER_GRIP_CLOSED, 100),
         Servo.FrontVacuumLift(FRONT_VACUUM_LIFT_PICKUP, 100)],
      a=[I_O.Pump(1), I_O.Valve(1)])

    s(s=[Servo.FrontVacuumLift(FRONT_VACUUM_LIFT_PLANKS, 100)])
    s(s=[Servo.FrontVacuum(FRONT_VACUUM_UPWARD_HOLDING, 100)])

    s(s=[Servo.FrontGripLift(FRONT_GRIP_LIFT_UP, 100), 
         Servo.CenterSwing(CENTER_SWING_UP, 10)])
    s(s=[Servo.CenterLift(CENTER_LIFT_LEVEL2, 100),
         Servo.FrontGripLift(FRONT_GRIP_LIFT_DOWN, 100)])

    s(s=[Servo.CenterSwing(CENTER_SWING_DOWN, 100),
         Servo.CenterLift(CENTER_LIFT_POSITIONING_CANS, 100)])

    s(s=[Servo.FrontVacuumLift(FRONT_VACUUM_LIFT_DOWN, 100), 
         Servo.FrontVacuum(FRONT_VACUUM_UPWARD_PLACING, 10)],
      a=[I_O.Pump(0), I_O.Valve(0)])

    s(m=Move.Distance(-150, 100, 100),
      s=[Servo.FrontVacuum(FRONT_VACUUM_OUTSTRETCHED, 100), 
         Servo.FrontSideGrip(FRONT_SIDE_GRIP_OPEN, 100),
         Servo.CenterLift(CENTER_LIFT_DROPPING_CANS, 10),
         Servo.FrontCenterGrip(FRONT_CENTER_GRIP_OPEN, 100)])
    
    return s.steps


def three_and_one_level():
    '''
        Separates one material stock into a third level and first level.
        The third level is lifted on top of a two level, and the first is in the back grippers.
    '''
    s = Strategy()

    s(m=Move.Distance(150, 100, 300))

    s(s=[Servo.FrontCenterGrip(FRONT_CENTER_GRIP_CLOSED, 100),
         Servo.FrontSideGrip(FRONT_SIDE_GRIP_CLOSED, 100)])

    s(s=[Servo.FrontVacuumLift(FRONT_VACUUM_LIFT_PICKUP, 100)],
      a=[I_O.Pump(1), I_O.Valve(1)])

    s(s=[Servo.FrontVacuumLift(250, 100), 
         Servo.FrontVacuum(240, 40)])

    s(s=[Servo.FrontGripLift(FRONT_GRIP_LIFT_UP, 100), 
         Servo.CenterSwing(CENTER_SWING_UP, 10)])
    s(s=[Servo.CenterLift(CENTER_LIFT_LEVEL2, 100),
         Servo.FrontGripLift(FRONT_GRIP_LIFT_DOWN, 100)])
    s(s=[Servo.FrontSideGrip(FRONT_SIDE_GRIP_OPEN, 100)])

    s(m=Move.Distance(-150, 100, 100))
    s(m=Move.Rotate(3.14, 1, 1), 
      s=[Servo.BackSideGrip(BACK_SIDE_GRIP_OPEN, 100)])
    s(m=Move.Distance(-170, 100, 100))
    s(s=[Servo.BackSideGrip(BACK_SIDE_GRIP_CLOSED, 100)])
    s(m=Move.Distance(150, 100, 100))
    s(m=Move.Rotate(3.14, 1, 1))

    s(s=[Servo.CenterSwing(CENTER_SWING_LEVEL3, 100),
         Servo.CenterLift(CENTER_LIFT_LEVEL3, 100)],
      a=[I_O.Pump(0), I_O.Valve(0)])

    s(s=[Servo.FrontVacuumLift(180, 100)])

    s(s=[Servo.FrontVacuum(FRONT_VACUUM_OUTSTRETCHED, 100),
         Servo.FrontVacuumLift(250, 100)])

    s(m=Move.Distance(350, 100, 100))

    s(s=[Servo.CenterSwing(CENTER_SWING_DOWN, 50), 
         Servo.FrontCenterGrip(FRONT_CENTER_GRIP_OPEN, 100)])

    s(m=Move.Distance(-300, 100, 100))

    return s.steps


def level_lift():
    '''
        Lifts one or two levels on top of a one level.
    '''
    s= Strategy()

    s(s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_OPEN, 100),
             Servo.BackSideGrip(BACK_SIDE_GRIP_OPEN, 100),
             Servo.BackLift(BACK_LIFT_DOWN, 50)])

    s(m=Move.Distance(-170, 100, 100))
    s(s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_CLOSED, 100),
         Servo.BackSideGrip(BACK_SIDE_GRIP_CLOSED, 100)])

    s(s=[Servo.BackLift(BACK_LIFT_UP, 50)])

    s(m=Move.Distance(-200, 100, 100))
    s(s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_OPEN, 100),
         Servo.BackSideGrip(BACK_CENTER_GRIP_OPEN, 100),
         Servo.BackLift(BACK_LIFT_UP-50, 50)])  # ZAGLAVICE SE NA KABEL OD SENZORA AKO SE NE STAVI -50

    s(m=Move.Distance(200, 100, 100))

    return s.steps

def all_grip():
    s = Strategy()

    s(s=[Servo.FrontSideGrip(50, 100),
         Servo.FrontCenterGrip(50, 100),
         Servo.BackCenterGrip(50, 100),
         Servo.BackSideGrip(50, 100)])

    return s.steps

def grip_front():
    s = Strategy()

    s(s=[Servo.FrontSideGrip(50, 100),
         Servo.FrontCenterGrip(130, 100)])

    return s.steps

def grip_back():
    s = Strategy()

    s(s=[Servo.BackCenterGrip(50, 100),
         Servo.BackSideGrip(50, 100)])

    return s.steps

def all_open():
    s = Strategy()

    s(s=[Servo.FrontSideGrip(30, 100),
         Servo.FrontCenterGrip(30, 100),
         Servo.BackCenterGrip(30, 100),
         Servo.BackSideGrip(40, 100)])

    return s.steps

def ungrip_back():
    s = Strategy()

    s(s=[Servo.BackCenterGrip(30, 100),
         Servo.BackSideGrip(40, 100)])

    return s.steps

def ungrip_front():
    s = Strategy()

    s(s=[Servo.FrontSideGrip(30, 100),
         Servo.FrontCenterGrip(150, 100)])

    return s.steps


