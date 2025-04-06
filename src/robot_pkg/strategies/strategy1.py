from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType
# from robot_pkg.play_elements import Area, MaterialStock

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
# strategy1(s=[Servo.FrontVacuumLift(150, 10)])
# strategy1(s=[Servo.FrontVacuum(60, 10)])
# strategy1(m=Move.Distance(-200, 100, 2000))

# strategy1(s=[Servo.BackCenterGrip(50,10),
#             Servo.BackSideGrip(130, 10),
#             Servo.BackLift(250, 10)])


strategy1(s=[Servo.FrontSideGrip(30, 100),
             Servo.FrontCenterGrip(150, 100),
             Servo.FrontGripLift(0, 50),
             Servo.FrontVacuumLift(0, 10),
             Servo.FrontVacuum(60, 100),
             Servo.CenterSwing(150, 10),
             Servo.CenterLift(300, 10)],) #, c=[(ConditionType.TIMEOUT, None, 3),])
# strategy1(m=Move.Distance(100, 100, 100))
strategy1(s=[Servo.FrontSideGrip(90, 100),
             Servo.FrontCenterGrip(90, 100)])
# strategy1(m=Move.Distance(50, 100, 100))

strategy1(s=[Servo.FrontVacuumLift(210, 50)],
          a=[I_O.Pump(1), 
             I_O.Valve(1)])

strategy1(s=[Servo.FrontVacuumLift(0, 20), 
            Servo.FrontVacuum(230, 10)])

strategy1(s=[Servo.FrontGripLift(250, 20)])
strategy1(s=[Servo.CenterSwing(240, 10)])
strategy1(s=[Servo.CenterLift(200, 10),
             Servo.FrontGripLift(0, 20)])

strategy1(s=[Servo.CenterSwing(150, 10),
             Servo.CenterLift(180, 10)])

strategy1(s=[Servo.FrontVacuumLift(200, 10)])
strategy1(a=[I_O.Pump(0), 
             I_O.Valve(0)])







# strategy1(s=[Servo.FrontVacuumLift(0, 10)])
            


# strategy1(s=[Servo.FrontVacuumLift(0, 100)])
# strategy1(s=[Servo.FrontSideGrip(180, 100)])



# strategy1(m=Move.Distance(100, 100, 100))

# strategy1(s=[Servo.])
# strategy1(s=[Servo.RightVacuumLift(0, 100), 
#              Servo.RightVacuum(60, 100),
#              Servo.LeftVacuumLift(300, 100),
#              Servo.LeftVacuum(240, 100)])

# strategy1(a=[I_O.Pump(1), 
#              I_O.Valve(1)])

# strategy1(s=[Servo.RightVacuumLift(210, 100), 
#              Servo.LeftVacuumLift(90, 100)])

# strategy1(s=[Servo.RightVacuumLift(0, 100), 
#              Servo.RightVacuum(235, 100),
#              Servo.LeftVacuumLift(300, 100),
#              Servo.LeftVacuum(65, 100)])

# strategy1(s=[Servo.RightVacuumLift(300, 100), 
#              Servo.LeftVacuumLift(0, 100)])

# strategy1(a=[I_O.Pump(0), 
#              I_O.Valve(0)])

# strategy1(s=[Servo.RightVacuum(150, 100),
#              Servo.LeftVacuum(150, 100)])

# strategy1(m=Move.Distance(-150, 200, 200))
