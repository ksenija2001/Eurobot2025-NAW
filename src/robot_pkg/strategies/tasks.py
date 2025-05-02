from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.misc import *
from robot_pkg.play_elements import *
from robot_pkg.consts import Points
import math

def init_position(init_x, init_y, init_theta, final_position:str):
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

     s(m=Move.ResetOdom(0, 0, angle),
          c=[Condition.CinchPulled(1)])

     s(ID=1, m=Move.Distance(150, 100, 100))
     # s(task_steps=init_all_servos())

     return s.steps

def open_front():
     s = Strategy()

     s(s=[Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideLeft.OPEN),
         Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN)])
     
     return s.steps

def close_front():
     s = Strategy()

     s(s=[Servo.FrontSideGrip(FrontSideLeft.CLOSED, FrontSideLeft.CLOSED),
         Servo.FrontCenterGrip(FrontCenterLeft.CLOSED, FrontCenterRight.CLOSED)])
     
     return s.steps

def open_back():
     s = Strategy()

     s(s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN), 
         Servo.BackCenterGrip(BackCenterLeft.OPEN, BackSideRight.OPEN)])
     
     return s.steps

def close_back():
     s = Strategy()

     s(s=[Servo.BackSideGrip(BackSideLeft.CLOSED, BackSideRight.CLOSED), 
         Servo.BackCenterGrip(BackCenterLeft.CLOSED, BackSideRight.CLOSED)])
     
     return s.steps

def init_all_servos():
    '''
        Brings all servos to their starting position.
        The starting position takes up the least amount of space.
    '''

    s = Strategy()
    s(s=[Servo.FrontSideGrip(FrontSideLeft.CLOSED, FrontSideRight.CLOSED),
         Servo.FrontCenterGrip(FrontCenterLeft.NEUTRAL, FrontCenterRight.NEUTRAL),
         Servo.BackSideGrip(BackSideLeft.CLOSED, BackSideRight.CLOSED),  # PROMENITI KADA SE ISEKU GRIPPERI
         Servo.BackCenterGrip(BackCenterLeft.CLOSED, BackCenterRight.CLOSED),
         Servo.FrontGripLift(FrontGripLift.DOWN),
         Servo.FrontVacuumLift(VacuumLift.UP),
         Servo.FrontVacuum(Vacuum.DOWN),
         Servo.CenterSwing(CenterSwing.INIT),
         Servo.CenterLift(CenterLift.DOWN, 50),
         Servo.BackLift(BackGripLift.HOLD)],
       a=[I_O.Pump(0), I_O.Valve(0)])
    
    return s.steps

def init_front_servos():
    '''
        Brings all front servos to pickup positions.
    '''

    s = Strategy()
    s(s=[Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
         Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN),
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
     s(s=[Servo.BackSideGrip(BackSideLeft.CLOSED, BackSideRight.CLOSED),
          Servo.BackCenterGrip(BackCenterLeft.CLOSED, BackCenterRight.CLOSED),
          Servo.BackLift(BACK_LIFT_DOWN)])

     return s.steps

def pickup_back_full_stack(distance=0):
     '''
        Picks-up and holds one stack with back servos.
        Backing out is not included.
     '''

     s = Strategy()

     s(m=Move.Distance(-200+distance, 1000, 300),
       s=[Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN),
          Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
          Servo.BackLift(BackGripLift.DOWN)])

     s(s=[Servo.BackCenterGrip(BackCenterLeft.GRIP, BackCenterRight.GRIP),
         Servo.BackSideGrip(BackSideLeft.GRIP, BackSideRight.GRIP)])

     s(s=[Servo.BackLift(BackGripLift.HOVER)])

     return s.steps

def pickup_front_full_stack(distance=0):
    '''
        Picks-up and holds one stack with front servos.
        Separating is not included.
        Backing out is not included.
    '''

    s = Strategy()
    s(m=Move.Distance(250+distance, 700, 300), # 300 SA RAZLOGOM
      s=[Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
         Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN),
         Servo.FrontVacuum(Vacuum.DOWN),
         Servo.CenterSwing(CenterSwing.DOWN, 50),
         Servo.CenterLift(CenterLift.DOWN),
         Servo.FrontGripLift(FrontGripLift.DOWN),
         Servo.FrontVacuumLift(VacuumLift.HOVER)])

    s(s=[Servo.FrontCenterGrip(FrontCenterLeft.GRIP, FrontCenterRight.GRIP),
         Servo.FrontSideGrip(FrontSideLeft.GRIP, FrontSideRight.GRIP),
         Servo.FrontVacuumLift(VacuumLift.PICKUP2)],
      a=[I_O.Pump(1), I_O.Valve(1)])
    
    s(s=[Servo.FrontGripLift(FrontGripLift.HOVER),
         Servo.FrontVacuumLift(VacuumLift.HOVER),
         Servo.CenterLift(CenterLift.HOVER)
         ])

    return s.steps

def two_level():
    '''
        Separates one stack in two mid air. 
        No moving included.
    '''

    s = Strategy()
    
    s(s=[Servo.FrontVacuumLift(VacuumLift.HOLD+40)])
    s(s=[Servo.FrontGripLift(100),
         Servo.CenterLift(CenterLift.DOWN)])

    s(s=[Servo.FrontVacuum(Vacuum.MIDDLE),
         Servo.FrontGripLift(FrontGripLift.HOLD),
         Servo.CenterSwing(CenterSwing.UP)
         ])

    s(s=[
          Servo.CenterLift(CenterLift.HOLD2),
         Servo.FrontGripLift(FrontGripLift.HOVER)])

    s(s=[
          Servo.CenterSwing(CenterSwing.DOWN, 30),
         Servo.FrontVacuum(Vacuum.UP, 30),
         Servo.CenterLift(CenterLift.POSITION2),
         Servo.FrontVacuumLift(VacuumLift.POSITION2+10, 50)])

    return s.steps

def drop_two_level(distance=0):
     '''
        Leaves a two level construction in place and backs out. 
        Backs out.
        Points: 12
     '''

     s = Strategy()

     s(s=[Servo.CenterLift(CenterLift.POSITION2+40)],
          a=[I_O.Pump(0), I_O.Valve(0)])
     
     s(s=[Servo.FrontVacuumLift(VacuumLift.POSITION2-20)])
     
     s(s=[Servo.FrontVacuum(Vacuum.PUSH),
          Servo.FrontVacuumLift(VacuumLift.DOWN),
          Servo.CenterLift(CenterLift.DROP2, 50),
          Servo.FrontGripLift(FrontGripLift.DOWN, 50)])
          
     s(m=Move.Distance(-200, 1000, 500),
          s=[Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
          Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN)],
          p = Points.LEVEL1+Points.LEVEL2)
     
     return s.steps

def drop_one_level(distance=0):
     '''
        Leaves a one level construction in place and backs out. 
        Backs out.
        Points: 4
     '''

     s = Strategy()
        
     s(m=Move.Distance(-150+distance, 1000, 500),
      s=[Servo.CenterLift(CenterLift.DROP1),
         Servo.FrontGripLift(FrontGripLift.DOWN),
         Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN)],
      p = Points.LEVEL1)
    
     return s.steps

def drop_separate_two_level(between_drop_distance=0, backout_distance=0):
     '''
          Drops lower level and moves back to drop second level.
          Backs out.
          Points: 8
     '''
     
     s = Strategy()

     s(task_steps=drop_one_level(between_drop_distance))

     s(s=[Servo.CenterLift(CenterLift.DOWN),
          Servo.FrontVacuumLift(VacuumLift.DROP1, 20),
          Servo.FrontVacuum(Vacuum.DOWN, 25)])

     s(s=[Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN)],
          a=[I_O.Pump(0), I_O.Valve(0)],
          p=Points.LEVEL1)

     s(m=Move.Distance(-200+backout_distance, 1000, 500),
          s=[Servo.FrontVacuumLift(VacuumLift.HOVER)])

     return s.steps

def lift_two_on_one(distance=0, back_distance=0):
     '''
        Places two levels on a one level high construction on the ground.
        Backs out.
        Points: 25
     '''

     s = Strategy()

     s(s=[Servo.CenterLift(CenterLift.LIFT2, 50)],
          a=[I_O.Pump(0), I_O.Valve(0)])

     # s(s=[Servo.CenterLift(CenterLift.UP, 60),
     #      Servo.FrontVacuumLift(VacuumLift.UP, 60),
     #      Servo.FrontGripLift(FrontGripLift.UP-20, 30),
     #      Servo.FrontVacuum(Vacuum.MIDDLE, 60)])
          
     s(m=Move.Distance(150+distance, 800, 300),
          s=[Servo.CenterLift(CenterLift.UP, 60),
          Servo.FrontVacuumLift(VacuumLift.UP, 60),
          Servo.FrontGripLift(FrontGripLift.UP-20, 50),
          Servo.FrontVacuum(Vacuum.DROP, 60)])
     s(m=Move.Distance(-250-back_distance, 1000, 1000),
     s=[Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
          Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN),
          Servo.FrontVacuumLift(VacuumLift.UP-20)],
          p = Points.LEVEL2+Points.LEVEL3)
     
     return s.steps

def push_two_level(push_distance=0, backout_distance=0):
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

     s(m=Move.Distance(150+push_distance, 500, 50),
     s=[
          # Servo.CenterLift(CenterLift.DOWN),
          Servo.FrontGripLift(FrontGripLift.DOWN), #HOVER+20),
          Servo.FrontVacuumLift(10),
          Servo.FrontVacuum(Vacuum.PUSH)])

     s(m=Move.Distance(-300+backout_distance, 1000, 1000))

     return s.steps

def lift_one_on_two(distance=0):
     '''
        Places one level on a two level high construction on the ground.
        Backs out.
        Points: 16
     '''

     s = Strategy()

     s(s=[
          Servo.CenterLift(CenterLift.UP, 50),
          Servo.CenterSwing(CenterSwing.DOWN+10, 60),
          Servo.FrontVacuumLift(VacuumLift.HOLD, 50)],
          a=[I_O.Pump(0), I_O.Valve(0)])

     s(m=Move.Distance(200+distance, 1000, 400),  # BILO 500 UBRZANJE
       s=[Servo.FrontVacuumLift(VacuumLift.UP-15, 50),
          Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN),
          Servo.FrontGripLift(FrontGripLift.DOWN),
          Servo.FrontVacuum(Vacuum.DROP)])
     
     s(s=[Servo.CenterLift(CenterLift.UP-10),
          Servo.CenterSwing(CenterSwing.DOWN),
          Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
          Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN)])

     s(m=Move.Distance(-250-distance, 1000, 500), p = Points.LEVEL3)
     
     return s.steps

def leave_banner(back_distance=0, forward_distance=0):
     '''
          Leave banner by hitting the back wall.
          Forwards out.
          Points: 20
     '''

     s = Strategy()

     s(m=Move.Rotate(3.14, 15, 10))

     s(m=Move.Distance(-250+back_distance, 500, 500))

     s(m=Move.Distance(125+forward_distance, 1000, 500),
       p=Points.BANNER)

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

def drop_back_one_level():
     '''
        Drop one level with back servos.
        Forwards out.
        Points: 4
     '''

     # blue3(m=Move.Distance(-300, 300, 300))
     # blue3(s=[Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN),
     #         Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
     #         Servo.FrontSideGrip(FrontSideLeft.CLOSED, FrontSideRight.CLOSED),
     #         Servo.BackLift(BackGripLift.UP-40)])
     
     pass

def drop_back_two_level():
     '''
        Drop two level construction with back servos.
        Forwards out.
        Points: 8
     '''
     
     pass
