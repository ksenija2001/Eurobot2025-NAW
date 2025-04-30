from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *
from robot_pkg.misc import *

twos = Strategy(color = Color.BLUE, square = Square.CENTER, mood = Mood.AGGRESSIVE)

twos(m=Move.ResetOdom(1780, 230, 1.57),
  task_steps=init_all_servos()) #,
        # c=[Condition.CinchPulled(1)])  

# twos(m=Move.To(MaterialStack.STACK10.x,
#                 MaterialStack.STACK10.y - 250,
#                 'f',
#                 2000, 2000, 15, 15))

# twos(task_steps=pickup_front_full_stack())

pose = Position(3000, 1200)
twos(m=Move.Spline([MaterialStack.STACK9.x    , 1500, MaterialStack.STACK10.x    , Area.BLUE_2.x], 
                   [MaterialStack.STACK9.y-150, 1300, MaterialStack.STACK10.y-100, Area.BLUE_2.y+50], 
                    [1.57, 0, -1.57, -1.57], 
                        800, 'f'),
               s=[Servo.CenterSwing(CenterSwing.DOWN),
                Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
                  Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN),
                    Servo.FrontCenterGrip(FrontCenterLeft.GRIP, FrontCenterRight.GRIP, activate_pose=pose),
                  Servo.FrontSideGrip(FrontSideLeft.GRIP, FrontSideRight.GRIP, activate_pose=pose),
                  Servo.FrontVacuumLift(300, activate_pose=pose),
                  Servo.FrontVacuum(Vacuum.MIDDLE, activate_pose=pose),
                  Servo.FrontGripLift(FrontGripLift.UP, activate_pose=pose),
                  Servo.CenterLift(CenterLift.POSITION2+20, 50, activate_pose=pose),
                  Servo.CenterSwing(CenterSwing.DOWN+10, 50, activate_pose=pose)] )

twos(m=Move.Distance(-200, 300, 300))

twos(s=[Servo.CenterSwing(CenterSwing.DOWN, 50),
         Servo.CenterLift(CenterLift.DOWN, 80),
         Servo.FrontGripLift(FrontGripLift.DOWN, 40)])

twos(s=[Servo.FrontVacuum(Vacuum.DOWN),
         Servo.FrontVacuumLift(VacuumLift.PICKUP2)],
         a=[I_O.Pump(1), I_O.Valve(1)])

twos(task_steps=two_level())
twos(task_steps=drop_two_level())