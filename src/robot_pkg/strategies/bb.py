from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *

################
## BLUE BASIC ##
################

bb = Strategy(color = Color.BLUE, square = Square.CENTER, mood = Mood.PASSIVE)

bb(m=Move.ResetOdom(1780, 230, 1.57),
          s=[Servo.FrontSideGrip(SideGripper.CLOSED),
            Servo.FrontCenterGrip(Gripper.NEUTRAL),
            Servo.BackSideGrip(Gripper.CLOSED),
            Servo.BackCenterGrip(Gripper.OPEN),
            Servo.FrontGripLift(FrontGripLift.DOWN),
            Servo.FrontVacuumLift(VacuumLift.UP),
            Servo.FrontVacuum(Vacuum.DOWN),
            Servo.CenterSwing(CenterSwing.INIT),
            Servo.CenterLift(CenterLift.DOWN),
            Servo.BackLift(BackGripLift.DOWN)]) #,
        # c=[Condition.CinchPulled(1)])   

####################
## LEAVING BANNER ##
####################
# bb(m=Move.Distance(-150, 1000, 1500))

##############################
## PICK-UP and DROP STACK 6 ##
##############################
bb(m=Move.Spline([MaterialStack.STACK6.x + 20],
                 [MaterialStack.STACK6.y + 250],
                 [-1.57],
                 400,
                 'f'))

bb(m=Move.RotateTo(-1.57, 15, 10))

bb(task_steps=pickup_front_full_stack())
bb(task_steps=two_level())
bb(task_steps=drop_two_level())

#####################
## PICK-UP STACK 7 ##
#####################

bb(m=Move.To(MaterialStack.STACK7.x-250, MaterialStack.STACK7.y, 'r', 1000, 1500, 15, 10))
bb(m=Move.RotateTo(3.14, 15, 10))

bb(task_steps=pickup_back_full_stack())

######################
## PICK-UP STACK 10 ##
######################

bb(m=Move.Spline([MaterialStack.STACK10.x + 100],
                 [MaterialStack.STACK10.y - 300],
                 [1.57],
                 500,
                 'f'),
  s=[Servo.FrontVacuumLift(VacuumLift.UP),
     Servo.CenterLift(CenterLift.DOWN)])

bb(m=Move.RotateTo(1.57, 15, 5))

bb(task_steps=pickup_front_full_stack())

############################
## SORT and DROP STACK 10 ##
############################

bb(m=Move.RotateTo(-1.57, 15, 5))

bb(task_steps=two_level())

bb(m=Move.To(Area.BLUE_2.x, Area.BLUE_2.y, 'f', 1000, 1000, 15, 5))
bb(m=Move.RotateTo(-1.57, 15, 5))

bb(task_steps=drop_one_level())

bb(m=Move.RotateTo(0, 15, 5))









