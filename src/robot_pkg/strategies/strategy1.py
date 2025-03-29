from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.step import Move, Servo, Actuator
from robot_pkg.conditions import ConditionType

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

# strategy1(m=Move.Distance(100, 100, 100))


strategy1(s=[Servo.RightVacuumLift(0, 100), 
            Servo.RightVacuum(60, 100),
            Servo.LeftVacuumLift(300, 100),
            Servo.LeftVacuum(240, 100)])

strategy1(a=[Actuator.Pump(1), Actuator.Valve(1)])

strategy1(s=[Servo.RightVacuumLift(210, 100), 
            Servo.LeftVacuumLift(90, 100)])

strategy1(s=[Servo.RightVacuumLift(0, 100), 
            Servo.RightVacuum(235, 100),
            Servo.LeftVacuumLift(300, 100),
            Servo.LeftVacuum(65, 100)])

strategy1(s=[Servo.RightVacuumLift(300, 100), 
            Servo.LeftVacuumLift(0, 100)])

strategy1(a=[Actuator.Pump(0), Actuator.Valve(0)])

strategy1(s=[Servo.RightVacuum(150, 100),
            Servo.LeftVacuum(150, 100)])

# strategy1(m=Move.Distance(-150, 200, 200))
