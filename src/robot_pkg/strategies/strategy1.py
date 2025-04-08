from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move
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

# strategy1(task_steps=init_servos())
# strategy1(task_steps=two_level())



# strategy1(task_steps=init_front_servos())


strategy1(s=[Servo.BackCenterGrip(30, 100),
             Servo.BackSideGrip(20, 100),
             Servo.BackLift(300, 50)])

strategy1(m=Move.Distance(-150, 100, 100))
strategy1(s=[Servo.BackCenterGrip(50, 100),
            Servo.BackSideGrip(50, 100)])

strategy1(s=[Servo.BackLift(0, 30)])

strategy1(m=Move.Distance(-150, 100, 100))
strategy1(s=[Servo.BackCenterGrip(30, 100),
            Servo.BackSideGrip(20, 100)])

strategy1(m=Move.Distance(150, 100, 100))






# strategy1(s=[Servo.FrontSideGrip(30, 100),
#              Servo.FrontCenterGrip(150, 100),
#              Servo.FrontGripLift(0, 100),
#              Servo.FrontVacuumLift(0, 100),
#              Servo.FrontVacuum(60, 100),
#              Servo.CenterSwing(150, 100),
#              Servo.CenterLift(300, 100)]) #, c=[(ConditionType.TIMEOUT, None, 3),])

# strategy1(m=Move.Distance(150, 100, 300))
# strategy1(s=[Servo.FrontSideGrip(90, 100),
#              Servo.FrontCenterGrip(90, 100)])
# # strategy1(m=Move.Distance(50, 100, 100))

# strategy1(s=[Servo.FrontVacuumLift(210, 100)],
#           a=[I_O.Pump(1), 
#              I_O.Valve(1)])

# strategy1(s=[Servo.FrontVacuumLift(100, 100), 
#             Servo.FrontVacuum(230, 100)])

# strategy1(s=[Servo.FrontGripLift(250, 100), Servo.CenterSwing(240, 10)])
# strategy1(s=[Servo.CenterLift(210, 100),
#              Servo.FrontGripLift(0, 100)])

# strategy1(s=[Servo.CenterSwing(150, 100),
#              Servo.CenterLift(160, 100)])

# strategy1(s=[Servo.FrontVacuumLift(300, 100), Servo.FrontVacuum(245, 100)],
#          a=[I_O.Pump(0), 
#              I_O.Valve(0)])

# strategy1(s=[Servo.FrontVacuum(150, 100), 
#             Servo.FrontSideGrip(30, 100),
#             Servo.CenterLift(180, 10),
#             Servo.FrontCenterGrip(150, 100)],
#          m=Move.Distance(-150, 100, 100))



