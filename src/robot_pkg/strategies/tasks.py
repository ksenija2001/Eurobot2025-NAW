from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType

def init_front_servos():
    '''
        Brings all front servos to their starting position
    '''
    s = Strategy()
    s(s=[Servo.FrontSideGrip(30, 100),
         Servo.FrontCenterGrip(150, 100),
         Servo.FrontGripLift(0, 100),
         Servo.FrontVacuumLift(0, 100),
         Servo.FrontVacuum(60, 100),
         Servo.CenterSwing(150, 100),
         Servo.CenterLift(300, 100)])

    return s.steps

def two_level():
    '''
        Build two levels from one material stock.
    '''
    s = Strategy()

    s(m=Move.Distance(150, 100, 300))
    s(s=[Servo.FrontSideGrip(90, 100),
         Servo.FrontCenterGrip(90, 100)])

    s(s=[Servo.FrontVacuumLift(210, 100)],
      a=[I_O.Pump(1), I_O.Valve(1)])

    s(s=[Servo.FrontVacuumLift(100, 100), 
         Servo.FrontVacuum(230, 100)])

    s(s=[Servo.FrontGripLift(250, 100), 
         Servo.CenterSwing(240, 10)])
    s(s=[Servo.CenterLift(210, 100),
         Servo.FrontGripLift(0, 100)])

    s(s=[Servo.CenterSwing(150, 100),
         Servo.CenterLift(160, 100)])

    s(s=[Servo.FrontVacuumLift(300, 100), 
         Servo.FrontVacuum(245, 100)],
      a=[I_O.Pump(0), I_O.Valve(0)])

    s(m=Move.Distance(-150, 100, 100),
      s=[Servo.FrontVacuum(150, 100), 
         Servo.FrontSideGrip(30, 100),
         Servo.CenterLift(180, 10),
         Servo.FrontCenterGrip(150, 100)])

def tree_and_one_level():
    '''
        Separates one material stock into a third level and first level.
        The third level is lifted on top of a two level, and the first is in the back grippers.
    '''
    s = Strategy()

    s(m=Move.Distance(150, 100, 300))
    s(s=[Servo.FrontCenterGrip(90, 100),
         Servo.FrontSideGrip(90, 100)])

    s(s=[Servo.FrontVacuumLift(220, 100)],
      a=[I_O.Pump(1), I_O.Valve(1)])

    s(s=[Servo.FrontVacuumLift(50, 100), 
         Servo.FrontVacuum(240, 40)])

    s(s=[Servo.FrontGripLift(250, 100), 
         Servo.CenterSwing(240, 10)])
    s(s=[Servo.CenterLift(210, 100),
         Servo.FrontGripLift(0, 100)])
    s(s=[Servo.FrontSideGrip(20, 100)])

    s(m=Move.Distance(-150, 100, 100))
    s(m=Move.Rotate(3.14, 1, 1), 
      s=[Servo.BackSideGrip(20, 100)])
    s(m=Move.Distance(-170, 100, 100))
    s(s=[Servo.BackSideGrip(90, 100)])
    s(m=Move.Distance(150, 100, 100))
    s(m=Move.Rotate(3.14, 1, 1))

    s(s=[Servo.CenterSwing(160, 100),
         Servo.CenterLift(0, 100)],
      a=[I_O.Pump(0), I_O.Valve(0)])

    s(s=[Servo.FrontVacuumLift(120, 100)])

    s(s=[Servo.FrontVacuum(150, 100),
         Servo.FrontVacuumLift(50, 100)])

    s(m=Move.Distance(350, 100, 100))

    s(s=[Servo.CenterSwing(150, 50), 
         Servo.FrontCenterGrip(150, 100)])

    s(m=Move.Distance(-300, 100, 100))



