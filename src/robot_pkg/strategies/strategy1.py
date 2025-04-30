from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
# from robot_pkg.play_elements import Area, MaterialStock
from robot_pkg.strategies.tasks import *

strategy1 = Strategy(color = Color.YELLOW, square = Square.LOWER, mood = Mood.PASSIVE)

strategy1(m=Move.ResetOdom(0,0,1.57))
# strategy1(task_steps=init_all_servos())
# strategy1(s=[Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN),
#               Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
#               Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
#               Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN)])

# strategy1(task_steps=two_level())
strategy1(s=[Servo.BackCenterGrip(BackCenterLeft.GRIP, BackCenterRight.GRIP),
              Servo.BackSideGrip(BackSideLeft.GRIP, BackSideRight.GRIP),
              Servo.FrontCenterGrip(FrontCenterLeft.GRIP, FrontCenterRight.GRIP),
              Servo.FrontSideGrip(FrontSideLeft.GRIP, FrontSideRight.GRIP)])


# strategy1(task_steps=pickup_front_full_stack())
# strategy1(m=Move.Distance(1000, 600, 300), task_steps=two_level())
# strategy1(task_steps=drop_two_level())
# strategy1(task_steps=pickup_front_full_stack())
# strategy1(task_steps=two_level())
# strategy1(task_steps=drop_two_level())
# strategy1(m=Move.Distance(-150, 1000, 1500))

# strategy1(m=Move.Distance(200, 500, 500))
# strategy1(m=Move.Distance(200, 500, 500))

# strategy1(m=Move.ResetOdom(1780, 230, 1.57))
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
# strategy1(task_steps=init_front_servos())
# strategy1(task_steps=level_lift())

# strategy1(s=[Servo.FrontSideGrip(FRONT_SIDE_GRIP_OPEN, 100),
#          Servo.FrontCenterGrip(FRONT_CENTER_GRIP_OPEN, 100)])
# strategy1(a=[I_O.Pump(1), I_O.Valve(1)])


# strategy1(m=Move.Distance(500, 100, 100),
#          s=[Servo.FrontCenterGrip(130, 100, Position(100, 250, 0))])

# strategy1(m=Move.Distance(1000, 1000, 1000))

# KOCKA
# strategy1(m=Move.To(0, 500, 'f', 1000, 1000, 10, 10))
# strategy1(m=Move.To(500, 500, 'f', 1000, 1000, 10, 10))
# strategy1(m=Move.To(500, 0, 'f', 1000, 1000, 10, 10))
# strategy1(m=Move.To(0, 0, 'f', 1000, 1000, 10, 10))
# strategy1(m=Move.RotateTo(1.57, 10, 10))


# strategy1(m=Move.Rotate(3.14, 15, 5))
# strategy1(task_steps=pickup_back_full_stack())
# strategy1(m=Move.Distance(150, 300, 500))
# strategy1(m=Move.Rotate(3.14, 15, 5))

# strategy1(task_steps=lift_one_on_two())
# strategy1(task_steps=lift_two_on_one())

# strategy1(task_steps=drop_two_level())
# strategy1(task_steps=init_back_servos())


# strategy1(s=[Servo.BackSideGrip(Gripper.CLOSED),
#              Servo.BackCenterGrip(Gripper.CLOSED),
#              Servo.FrontCenterGrip(Gripper.CLOSED),
#              Servo.FrontSideGrip(Gripper.CLOSED)],
#           a=[I_O.Pump(0), I_O.Valve(0)])


## TEST
# strategy1(m=Move.To(2600, 1330, 'f', 1000, 1500, 15, 10))
# strategy1(m=Move.Spline([2600], [1350], [0], 700, 'f'))
# strategy1(m=Move.RotateTo(0, 15, 10))

# strategy1(m=Move.Distance(240, 300, 300),
#       s=[Servo.FrontSideGrip(FRONT_SIDE_GRIP_OPEN, 100),
#          Servo.FrontCenterGrip(FRONT_CENTER_GRIP_OPEN, 100)])

# strategy1(task_steps=two_level())

# strategy1(m=Move.Distance(-240, 300, 300))

# strategy1(m=Move.To(3000-780, 2000-600, 'r', 1000, 1000, 15, 10))
# # strategy1(m=Move.Spline([3000-825], [2000-400-200], [-1.57], 500, 'r'))
# strategy1(m=Move.RotateTo(-1.57, 15, 10))

# strategy1(s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_OPEN, 100),
#              Servo.BackSideGrip(BACK_SIDE_GRIP_OPEN, 100),
#              Servo.BackLift(BACK_LIFT_DOWN, 30)])

# strategy1(m=Move.Distance(-240, 100, 100))
# strategy1(s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_CLOSED, 100),
#          Servo.BackSideGrip(BACK_SIDE_GRIP_CLOSED, 100)])

# strategy1(s=[Servo.BackLift(50, 30)])

# strategy1(m=Move.To(3000-780, 1000, 'f', 1000, 1000, 15, 3))
# strategy1(m=Move.To(1750, 400, 'r', 1000, 1000, 15, 3))
# strategy1(m=Move.RotateTo(1.57, 15, 10))

# strategy1(s=[Servo.BackLift(BACK_LIFT_DOWN, 30)])

# strategy1(s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_OPEN, 100),
#          Servo.BackSideGrip(BACK_SIDE_GRIP_OPEN, 100)])

# strategy1(m=Move.Distance(250, 300, 300))

# strategy1(m=Move.RotateTo(-1.57, 15, 3))

# strategy1(s=[Servo.FrontVacuumLift(50, 100), Servo.FrontVacuum(FRONT_VACUUM_OUTSTRETCHED, 100)])
# strategy1(s=[Servo.FrontGripLift(300, 100), 
#             Servo.CenterLift(300, 100), 
#             Servo.FrontVacuumLift(300, 100)])
          
# strategy1(m=Move.Distance(250, 300, 300))

# strategy1(s=[Servo.FrontCenterGrip(FRONT_CENTER_GRIP_OPEN, 100),
#             Servo.FrontSideGrip(FRONT_SIDE_GRIP_OPEN, 100)])

# strategy1(m=Move.Distance(-350, 300, 300))



  #    a=[I_O.Pump(1), I_O.Valve(1)])




# MOVEMENT TEST
# strategy1(m=Move.ResetOdom(230, 880, 0))
# strategy1(task_steps=init_all_servos())

# strategy1(m=Move.Spline([800, 1900, 2400, 2160], 
#                         [650, 650, 1000, 1500], 
#                         [0, 0, 1.57, 1.57], 
#                         1000, 
#                         'f'))

# strategy1(task_steps=init_all_servos())
# strategy1(task_steps=pickup_front_full_stack())
# strategy1(task_steps=two_level())
# # strategy1(task_steps=drop_one_level())
# # strategy1(m=Move.Rotate(3.14, 15, 5))
# # strategy1(task_steps=pickup_back_full_stack())
# # strategy1(m=Move.Rotate(3.14, 15, 5))
# # strategy1(task_steps=lift_one_on_two())
# # strategy1(task_steps=two_level())
# strategy1(task_steps=lift_two_on_one())

## ALEKSA REKAO MORA - DIZANJE  JEDAN NA DVA POD UGLOM

# strategy1(task_steps=init_all_servos())
# strategy1(task_steps=pickup_front_full_stack())
# strategy1(task_steps=two_level())
# strategy1(task_steps=drop_two_level())
# strategy1(m=Move.Distance(-200, 300, 300))
# strategy1(m=Move.Rotate(3.14, 15, 5))
# strategy1(task_steps=pickup_back_full_stack())
# strategy1(m=Move.Rotate(3.14, 15, 5))
# strategy1(task_steps=lift_one_on_two())
