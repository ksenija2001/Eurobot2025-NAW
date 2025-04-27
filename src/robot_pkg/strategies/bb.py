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
                 450,
                 'f'),
    s=[Servo.CenterSwing(CenterSwing.DOWN),
      Servo.FrontCenterGrip(Gripper.OPEN),
      Servo.FrontSideGrip(SideGripper.OPEN)])

bb(m=Move.RotateTo(-1.57, 15, 15))

bb(task_steps=pickup_front_full_stack(20))
bb(task_steps=two_level())
bb(task_steps=drop_two_level())

#####################
## PICK-UP STACK 7 ##
#####################

bb(m=Move.To(MaterialStack.STACK7.x-320, MaterialStack.STACK7.y, 'r', 1500, 1500, 15, 15))
bb(m=Move.RotateTo(3.14, 15, 15))

bb(task_steps=pickup_back_full_stack())
bb(s=[Servo.BackLift(BackGripLift.UP)])

######################
## PICK-UP STACK 10 ##
######################

bb(m=Move.Spline([MaterialStack.STACK10.x + 30],
                 [MaterialStack.STACK10.y - 275],
                 [1.57],
                 400,
                 'f'),
  s=[Servo.FrontVacuumLift(VacuumLift.UP),
     Servo.CenterLift(CenterLift.DOWN)])

bb(m=Move.RotateTo(1.57, 15, 10))

bb(task_steps=pickup_front_full_stack(30))

############################
## SORT and DROP STACK 10 ##
############################

bb(m=Move.RotateTo(-1.57, 15, 10))

bb(m=Move.To(Area.BLUE_2.x, Area.BLUE_2.y + 50, 'f', 1000, 1000, 15, 10),
  task_steps=two_level())

#bb(m=Move.To(Area.BLUE_2.x, Area.BLUE_2.y + 50, 'f', 1000, 1000, 15, 10))
bb(m=Move.RotateTo(-1.57, 15, 10))

bb(task_steps=drop_one_level(-150))

bb(m=Move.RotateTo(0, 15, 10))

#########################################
##  DROPPING STACK 7 FROM BACK GRIPPER ##
#########################################

bb(s=[Servo.BackLift(BackGripLift.DOWN, 30)])
bb(m=Move.Distance(200, 1500, 1500),
  s=[Servo.BackSideGrip(Gripper.OPEN),
     Servo.BackCenterGrip(Gripper.OPEN)])

bb(m=Move.To(MaterialStack.STACK6.x+25, MaterialStack.STACK6.y+200, 'f', 1500, 1500, 15, 15))
bb(m=Move.RotateTo(-1.57, 15, 15))

bb(task_steps=lift_one_on_two(70))

bb(m=Move.RotateTo(3.0, 15, 15))
bb(task_steps=(pickup_front_full_stack(450)))
bb(m=Move.To(Area.BLUE_2.x, Area.BLUE_2.y + 250, 'r', 1500, 1500, 15, 10),
  task_steps=two_level())

bb(m=Move.RotateTo(-1.57, 15, 15))

bb(task_steps=lift_two_on_one(50))

############################
##  PICKUP STACK 1   ##
############################

# TODO SPUSTI LIFTOVE
bb(m=Move.To(MaterialStack.STACK1.x+20, MaterialStack.STACK1.y - 350, 'r', 1500, 1500, 15, 15),
  s=[Servo.FrontVacuumLift(VacuumLift.UP),
     Servo.CenterLift(CenterLift.DOWN),
     Servo.FrontVacuum(Vacuum.DOWN),
     Servo.FrontGripLift(FrontGripLift.DOWN)])

bb(m=Move.RotateTo(-1.57, 15, 15))
bb(task_steps=pickup_back_full_stack())
bb(s=[Servo.BackLift(BackGripLift.UP)])

bb(m=Move.Distance(200, 1500, 1000))

# TODO SPLINE
bb(m=Move.To(MaterialStack.STACK8.x - 350, MaterialStack.STACK8.y, 'f', 1500, 1000, 15, 5))
bb(m=Move.RotateTo(0, 15, 10))
bb(task_steps=pickup_front_full_stack())

bb(m=Move.Distance(-200, 1500, 500))

bb(m=Move.To(Area.BLUE_2.x+20, Area.BLUE_2.y + 250, 'f', 1500, 1500, 15, 10),
    task_steps=two_level())
  
bb(m=Move.RotateTo(-1.57, 15, 10))

bb(task_steps=drop_two_level())

# bb(m=Move.RotateTo( ))

# bb(m=Move.Spline([],
#                 [],
#                 [],
#                 500, 
#                 'f'))












