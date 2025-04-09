from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType
# from robot_pkg.play_elements import Area, MaterialStock
from robot_pkg.strategies.tasks import *

strategy1 = Strategy(color = Color.YELLOW, square = Square.LOWER, mood = Mood.PASSIVE)

# strategy1(m=SetPosition(316, 305, 0), 
#             s=[Servo.Fork(ForkUp), 
#                   Servo.ForkLift(ForkLiftDown),
#                   Servo.Arm(ArmInside),
#                   Servo.Vacuum(VacuumCenter), 
#                   Servo.BucketLift(BucketLiftDown), 
#                   Servo.BucketHolder(BucketHolderUp), 
#                   Servo.BucketSeparator(BucketSeparatorHome)],                  
#           p = 5,
#           c =[(ConditionType.CINCH, 1),])


# strategy1(a=[I_O.Pump(0), I_O.Valve(0)])  # Check why not continuing

# strategy1(task_steps=grip_back())
strategy1(task_steps=init_front_servos())
strategy1(task_steps=tree_and_one_level())

strategy1(s=[Servo.FrontSideGrip(FRONT_SIDE_GRIP_OPEN, 100),
         Servo.FrontCenterGrip(FRONT_CENTER_GRIP_OPEN, 100)])
# strategy1(a=[I_O.Pump(1), I_O.Valve(1)])

# strategy1(m=Move.ResetOdom(0, 0, 1.57))

# strategy1(m=Move.Distance(500, 100, 100),
#          s=[Servo.FrontCenterGrip(130, 100, Position(100, 250, 0))])


