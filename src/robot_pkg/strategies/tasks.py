from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType
from robot_pkg.misc import *
from robot_pkg.play_elements import *
from robot_pkg.consts import Points

def init_all_servos():
    '''
        Brings all servos to their starting position.
        The starting position takes up the least amount of space.
    '''

    s = Strategy()
    s(s=[Servo.FrontSideGrip(Gripper.CLOSED),
         Servo.FrontCenterGrip(Gripper.CLOSED),
         Servo.BackSideGrip(Gripper.CLOSED),
         Servo.BackCenterGrip(Gripper.CLOSED),
         Servo.FrontGripLift(FRONT_GRIP_LIFT_DOWN),
         Servo.FrontVacuumLift(FRONT_VACUUM_LIFT_UP),
         Servo.FrontVacuum(FRONT_VACUUM_DOWNWARD),
         Servo.CenterSwing(CENTER_SWING_DOWN),
         Servo.CenterLift(CENTER_LIFT_DOWN),
         Servo.BackLift(BACK_LIFT_DOWN)],
       a=[I_O.Pump(0), I_O.Valve(0)])
    
    return s.steps

def init_front_servos():
    '''
        Brings all front servos to their starting position.
    '''

    s = Strategy()
    s(s=[Servo.FrontSideGrip(Gripper.CLOSED),
         Servo.FrontCenterGrip(Gripper.CLOSED),
         Servo.FrontGripLift(FRONT_GRIP_LIFT_DOWN),
         Servo.FrontVacuumLift(FRONT_VACUUM_LIFT_UP),
         Servo.FrontVacuum(FRONT_VACUUM_DOWNWARD),
         Servo.CenterSwing(CENTER_SWING_DOWN),
         Servo.CenterLift(CENTER_LIFT_DOWN)],
       a=[I_O.Pump(0), I_O.Valve(0)])

    return s.steps

def init_back_servos():
     '''
          Brings all back servos to their starting position.
     '''

     s = Strategy()
     s(s=[Servo.BackSideGrip(Gripper.CLOSED),
          Servo.BackCenterGrip(Gripper.CLOSED),
          Servo.BackLift(BACK_LIFT_DOWN)])

     return s.steps

def pickup_back_full_stack():
     '''
        Picks-up and holds one stack with back servos.
        Backing out is not included.
     '''

     s = Strategy()

     s(m=Move.Distance(-250, 300, 300),
       s=[Servo.BackCenterGrip(Gripper.OPEN),
          Servo.BackSideGrip(Gripper.OPEN),
          Servo.BackLift(BackGripLift.DOWN)])

     s(s=[Servo.BackCenterGrip(Gripper.GRIP),
         Servo.BackSideGrip(Gripper.GRIP)])

     s(s=[Servo.BackLift(BackGripLift.HOVER)])

     return s.steps

def pickup_front_full_stack():
    '''
        Picks-up and holds one stack with front servos.
        Separating is not included.
        Backing out is not included.
    '''

    s = Strategy()
    s(m=Move.Distance(250, 300, 300),
      s=[Servo.FrontCenterGrip(Gripper.OPEN),
         Servo.FrontSideGrip(Gripper.OPEN),
         Servo.FrontVacuum(Vacuum.DOWN)])

    s(s=[Servo.FrontCenterGrip(Gripper.GRIP),
         Servo.FrontSideGrip(Gripper.GRIP),
         Servo.FrontVacuumLift(VacuumLift.PICKUP2)],
      a=[I_O.Pump(1), I_O.Valve(1)])
    
    s(s=[Servo.FrontGripLift(FrontGripLift.HOVER),
         Servo.FrontVacuumLift(VacuumLift.HOVER),
         Servo.CenterLift(CenterLift.HOVER)])

    return s.steps

def two_level():
    '''
        Separates one stack in two mid air. 
        No moving included.
    '''

    s = Strategy()

    s(s=[Servo.FrontVacuumLift(VacuumLift.HOLD + 40),
         Servo.FrontGripLift(100, 60),
         Servo.CenterLift(CenterLift.DOWN, 50)])

    s(s=[Servo.FrontVacuum(Vacuum.MIDDLE),
         Servo.FrontGripLift(FrontGripLift.HOLD),
         Servo.CenterSwing(CenterSwing.UP)])

    s(s=[Servo.CenterLift(CenterLift.HOLD2),
         Servo.FrontGripLift(FrontGripLift.HOVER)])

    s(s=[Servo.CenterSwing(CenterSwing.DOWN, 30),
         Servo.FrontVacuum(Vacuum.UP, 50),
         Servo.CenterLift(CenterLift.POSITION2),
         Servo.FrontVacuumLift(VacuumLift.POSITION2+20, 50)])

    return s.steps

def drop_two_level():
     '''
        Leaves a two level construction in place and backs out. 
        Backs out.
        Points: 12
     '''

     # TODO works pretty bad
     s = Strategy()

     s(s=[Servo.CenterLift(CenterLift.POSITION2+20)],
          a=[I_O.Pump(0), I_O.Valve(0)])
     
     s(s=[Servo.FrontVacuum(Vacuum.MIDDLE),
          Servo.FrontVacuumLift(VacuumLift.DOWN)])
     s(s=[Servo.CenterLift(CenterLift.DROP2),
          Servo.FrontGripLift(FrontGripLift.DOWN)])
          
     s(m=Move.Distance(-250, 300, 500),
          s=[Servo.FrontCenterGrip(Gripper.OPEN),
          Servo.FrontSideGrip(Gripper.OPEN)],
          p = Points.LEVEL1+Points.LEVEL2)
     
     return s.steps

def drop_one_level():
     '''
        Leaves a one level construction in place and backs out. 
        Backs out.
        Points: 4
     '''

     s = Strategy()
        
     s(m=Move.Distance(-150, 300, 500),
      s=[Servo.FrontGripLift(FrontGripLift.DOWN),
         Servo.FrontSideGrip(Gripper.OPEN)],
      p = Points.LEVEL1)
    
     return s.steps

def lift_two_on_one():
     '''
        Places two levels on a one level high construction on the ground.
        Backs out.
        Points: 25
     '''

     s = Strategy()

     s(s=[Servo.CenterLift(CenterLift.LIFT2, 50)],
          a=[I_O.Pump(0), I_O.Valve(0)])

     s(s=[Servo.CenterLift(CenterLift.UP, 60),
          Servo.FrontVacuumLift(VacuumLift.UP, 60),
          Servo.FrontGripLift(FrontGripLift.UP, 60),
          Servo.FrontVacuum(Vacuum.MIDDLE, 60)])
          
     s(m=Move.Distance(150, 300, 500))
     s(m=Move.Distance(-250, 300, 500),
          s=[Servo.FrontCenterGrip(Gripper.OPEN),
          Servo.FrontSideGrip(Gripper.OPEN)],
          p = Points.LEVEL2+Points.LEVEL3)
     
     return s.steps

def lift_one_on_two():
     '''
        Places one level on a two level high construction on the ground.
        Backs out.
        Points: 16
     '''

     s = Strategy()

     s(s=[Servo.CenterLift(CenterLift.UP),
          Servo.CenterSwing(CenterSwing.DOWN+10, 60),
          Servo.FrontVacuumLift(VacuumLift.HOLD)],
          a=[I_O.Pump(0), I_O.Valve(0)])

     s(m=Move.Distance(200, 300, 500),
       s=[Servo.FrontVacuumLift(VacuumLift.UP, 50),
          Servo.FrontSideGrip(Gripper.OPEN),
          Servo.FrontGripLift(FrontGripLift.DOWN),
          Servo.FrontVacuum(Vacuum.MIDDLE)])

     s(m=Move.Distance(-250, 300, 500),
       s=[Servo.CenterSwing(CenterSwing.DOWN),
          Servo.FrontCenterGrip(Gripper.OPEN),
          Servo.FrontSideGrip(Gripper.OPEN)],
       p = Points.LEVEL3)
     
     return s.steps

def back_lift_one_on_one():
     '''
        Places one level on a one level high construction on the ground.
        Backs out.
        Points: 8
     '''
     
     pass

def back_lift_one_on_stack():
     '''
        Places one level on an untouched stack (with four cans and two planks) 
        on the ground.
        Pushes the untouched stack into an area.
        Points: 12
     '''

     pass



# def three_and_one_level():
#     '''
#         Separates one material stock into a third level and first level.
#         The third level is lifted on top of a two level, and the first is in the back grippers.
#     '''
#     s = Strategy()

#     s(m=Move.Distance(150, 100, 300))

#     s(s=[Servo.FrontCenterGrip(FRONT_CENTER_GRIP_CLOSED, 100),
#          Servo.FrontSideGrip(FRONT_SIDE_GRIP_CLOSED, 100)])

#     s(s=[Servo.FrontVacuumLift(FRONT_VACUUM_LIFT_PICKUP, 100)],
#       a=[I_O.Pump(1), I_O.Valve(1)])

#     s(s=[Servo.FrontVacuumLift(250, 100), 
#          Servo.FrontVacuum(240, 40)])

#     s(s=[Servo.FrontGripLift(FRONT_GRIP_LIFT_UP, 100), 
#          Servo.CenterSwing(CENTER_SWING_UP, 10)])
#     s(s=[Servo.CenterLift(CENTER_LIFT_LEVEL2, 100),
#          Servo.FrontGripLift(FRONT_GRIP_LIFT_DOWN, 100)])
#     s(s=[Servo.FrontSideGrip(FRONT_SIDE_GRIP_OPEN, 100)])

#     s(m=Move.Distance(-150, 100, 100))
#     s(m=Move.Rotate(3.14, 1, 1), 
#       s=[Servo.BackSideGrip(BACK_SIDE_GRIP_OPEN, 100)])
#     s(m=Move.Distance(-170, 100, 100))
#     s(s=[Servo.BackSideGrip(BACK_SIDE_GRIP_CLOSED, 100)])
#     s(m=Move.Distance(150, 100, 100))
#     s(m=Move.Rotate(3.14, 1, 1))

#     s(s=[Servo.CenterSwing(CENTER_SWING_LEVEL3, 100),
#          Servo.CenterLift(CENTER_LIFT_LEVEL3, 100)],
#       a=[I_O.Pump(0), I_O.Valve(0)])

#     s(s=[Servo.FrontVacuumLift(180, 100)])

#     s(s=[Servo.FrontVacuum(FRONT_VACUUM_OUTSTRETCHED, 100),
#          Servo.FrontVacuumLift(250, 100)])

#     s(m=Move.Distance(350, 100, 100))

#     s(s=[Servo.CenterSwing(CENTER_SWING_DOWN, 50), 
#          Servo.FrontCenterGrip(FRONT_CENTER_GRIP_OPEN, 100)])

#     s(m=Move.Distance(-300, 100, 100))

#     return s.steps


# def level_lift():
#     '''
#         Lifts one or two levels on top of a one level.
#     '''
#     s= Strategy()

#     s(s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_OPEN, 100),
#              Servo.BackSideGrip(BACK_SIDE_GRIP_OPEN, 100),
#              Servo.BackLift(BACK_LIFT_DOWN, 50)])

#     s(m=Move.Distance(-170, 100, 100))
#     s(s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_CLOSED, 100),
#          Servo.BackSideGrip(BACK_SIDE_GRIP_CLOSED, 100)])

#     s(s=[Servo.BackLift(BACK_LIFT_UP, 50)])

#     s(m=Move.Distance(-200, 100, 100))
#     s(s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_OPEN, 100),
#          Servo.BackSideGrip(BACK_CENTER_GRIP_OPEN, 100),
#          Servo.BackLift(BACK_LIFT_UP-50, 50)])  # ZAGLAVICE SE NA KABEL OD SENZORA AKO SE NE STAVI -50

#     s(m=Move.Distance(200, 100, 100))

#     return s.steps



