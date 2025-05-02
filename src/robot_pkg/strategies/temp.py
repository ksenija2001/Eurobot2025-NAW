from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *

temp = Strategy(color = Color.BLUE, 
                square = Square.CENTER, 
                mood = Mood.RUDE)

#######################################
## STARTING POSITION: BLUE HOME AREA ##
#######################################

temp(m=Move.ResetOdom(230, 922.5, 0),
        task_steps=init_all_servos())

# STACK 10
temp(m=Move.Spline([800, MaterialStack.STACK10.x-10],
                  [MaterialStack.STACK10.y+250, MaterialStack.STACK10.y+350],
                  [0.1, 0],
                  800,
                  'f'))

temp(m=Move.RotateTo(-1.57, 15, 10),
    s=[Servo.BackSideGrip(BackSideLeft.CLOSED, BackSideRight.CLOSED),
        Servo.BackCenterGrip(BackCenterLeft.CLOSED, BackCenterRight.CLOSED)])
temp(task_steps=pickup_front_full_stack())

# STACK 9

temp(m=Move.Spline([MaterialStack.STACK9.x-20],
                    [MaterialStack.STACK9.y+220],
                    [1.57],
                    500,
                    'r'))

temp(task_steps=pickup_back_full_stack())

# BLUE AREA 2

temp(m=Move.Spline([Area.BLUE_2.x-50],
                    [Area.BLUE_2.y],
                    [1.57],
                    500,
                    'r'),
    s=[Servo.BackLift(BackGripLift.UP, 50)])

# temp(m=Move.Spline([MaterialStack.STACK6.x],
#                     [MaterialStack.STACK6.y+150],
#                     [1.57],
#                     500,
#                     'r'),
#     s=[Servo.BackLift(BackGripLift.UP, 50)])

# temp(m=Move.Distance(-100, 300, 300))

temp(s=[Servo.BackLift(BackGripLift.DOWN, 50)])
temp(m=Move.Distance(300, 1000, 500),
      s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
        Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN)])

temp(m=Move.RotateTo(-1.57, 15, 5),
    task_steps=two_level())
# temp(m=Move.Distance(100, 300, 300))
temp(task_steps=drop_two_level())

# STACK 8

temp(m=Move.Spline([MaterialStack.STACK8.x-300],
                    [MaterialStack.STACK8.y],
                    [3.14],
                    600,
                    'r'),
    task_steps=init_front_servos())

temp(task_steps=pickup_back_full_stack(-50))

# STACK 1

temp(m=Move.Spline([MaterialStack.STACK1.x],
                    [MaterialStack.STACK1.y-350],
                    [1.57],
                    400,
                    'f'),
    s=[Servo.BackLift(BackGripLift.UP, 50),
        Servo.FrontSideGrip(FrontSideLeft.NEUTRAL, FrontSideRight.NEUTRAL),])

temp(task_steps=pickup_front_full_stack())

temp(m=Move.Distance(-400, 1500, 1500))

temp(m=Move.RotateTo(3.14, 15, 10))

# BLUE AREA 3

temp(m=Move.Spline([Area.BLUE_3.x],
                    [Area.BLUE_3.y],
                    [3.14],
                    1000,
                    'f'),
    task_steps=two_level())

temp(task_steps=drop_two_level())

temp(m=Move.RotateTo(1.57, 15, 5),
    s=[Servo.BackLift(BackGripLift.DOWN, 50)],
    task_steps=init_front_servos())

pose = Position(3000, 1000)
temp(m=Move.To(MaterialStack.STACK3.x+425, MaterialStack.STACK3.y-10, 'f', 1000, 500, 10, 5),
     s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN, pose),
        Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN, pose)])

temp(m=Move.RotateTo(0, 15, 15))

temp(task_steps=pickup_back_full_stack(-170))

temp(m=Move.Distance(250, 500, 500))

temp(m=Move.RotateTo(-1.57, 10, 5))

pose = Position(3000, 1450)
temp(s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN, pose),
        Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN, pose)])

temp(task_steps=pickup_front_full_stack(100))

temp(m=Move.To(Area.BLUE_3.x+150, Area.BLUE_3.y-180, 'f', 1500, 1000, 10, 5),
    task_steps=two_level())

temp(m=Move.RotateTo(-1.57, 10, 5))

temp(task_steps=drop_one_level())

temp(m=Move.RotateTo(3.14, 10, 5))
temp(task_steps=lift_one_on_two())

temp(m=Move.RotateTo(1.57, 15, 10))

temp(task_steps=pickup_back_full_stack())

pose = Position(3000, 1000)
temp(m=Move.Distance(400, 500, 500),
      s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN, pose),
        Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN, pose)])

temp(task_steps=pickup_front_full_stack())

temp(m=Move.RotateTo(-1.57, 10, 5),
    task_steps=two_level())

temp(task_steps=lift_two_on_one(100))



