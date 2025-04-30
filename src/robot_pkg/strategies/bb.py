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
  task_steps=init_all_servos()) #,
        # c=[Condition.CinchPulled(1)])   

####################
## LEAVING BANNER ##
####################
# bb(m=Move.Distance(-100, 1000, 1500))
# bb(m=Move.Distance(100, 300, 300))

##############################
## PICK-UP and DROP STACK 6 ##
##############################
bb(m=Move.Spline([MaterialStack.STACK6.x + 10],
                 [MaterialStack.STACK6.y + 250],
                 [-1.57],
                 500,
                 'f'),
    s=[
      Servo.CenterSwing(CenterSwing.DOWN),
      Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
      Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN)])

bb(m=Move.RotateTo(-1.57, 5, 5))

bb(task_steps=pickup_front_full_stack(20))
bb(task_steps=two_level())
bb(task_steps=drop_two_level())

#####################
## PICK-UP STACK 7 ##
#####################

bb(m=Move.To(MaterialStack.STACK7.x - 290, 
             MaterialStack.STACK7.y - 15, 
             'r', 1500, 2000, 15, 20))
bb(m=Move.RotateTo(3.14, 15, 15))

bb(task_steps=pickup_back_full_stack())
# bb(s=[Servo.BackLift(BackGripLift.UP)])

######################
## PICK-UP STACK 10 ##
######################

bb(m=Move.Spline([MaterialStack.STACK10.x + 10],
                 [MaterialStack.STACK10.y - 260],
                 [1.57],
                 500,
                 'f'),
  s=[Servo.BackLift(BackGripLift.UP),
     Servo.FrontVacuumLift(VacuumLift.UP),
     Servo.FrontVacuum(Vacuum.DOWN),
     Servo.CenterLift(CenterLift.DOWN),
     Servo.CenterSwing(CenterSwing.DOWN)
     ])

# bb(m=Move.RotateTo(1.57, 15, 15))

bb(task_steps=pickup_front_full_stack(30))

############################
## SORT and DROP STACK 10 ##
############################

# bb(m=Move.RotateTo(-1.57, 15, 5))

bb(m=Move.To(Area.BLUE_2.x, Area.BLUE_2.y + 50, 'f', 1000, 1000, 10, 3),
  task_steps=two_level())

#bb(m=Move.To(Area.BLUE_2.x, Area.BLUE_2.y + 50, 'f', 1000, 1000, 15, 10))
bb(m=Move.RotateTo(-1.57, 15, 10))

bb(task_steps=drop_one_level(-150))

#########################################
##  DROPPING STACK 7 FROM BACK GRIPPER ##
#########################################

bb(m=Move.RotateTo(0, 10, 5),
  s=[Servo.BackLift(BackGripLift.DOWN, 30)])

# bb(s=[Servo.BackLift(BackGripLift.DOWN, 30)])
# bb(m=Move.Distance(100, 1500, 1500),
#   s=[Servo.BackSideGrip(Gripper.OPEN),
#      Servo.BackCenterGrip(Gripper.OPEN)])

#########################################
## MOVE TO STACK 6 AND LIFT ONE ON TWO ##
#########################################
bb(m=Move.Spline([MaterialStack.STACK6.x + 30],
                 [MaterialStack.STACK6.y + 200],
                 [-1.57],
                 450, # 350
                 'f'),
    s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
     Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN)]
)
# bb(m=Move.To(MaterialStack.STACK6.x+25, MaterialStack.STACK6.y+200, 'f', 1500, 1500, 15, 5))
# bb(m=Move.RotateTo(-1.57, 15, 5))

bb(task_steps=lift_one_on_two(50))

bb(m=Move.RotateTo(3.0, 15, 15))
bb(task_steps=(pickup_front_full_stack(500)))
bb(m=Move.To(Area.BLUE_2.x, Area.BLUE_2.y + 250, 'f', 1500, 1000, 10, 3), # changed from r
  task_steps=two_level())

bb(m=Move.RotateTo(-1.57, 15, 5))

bb(task_steps=lift_two_on_one(50))

############################
##  PICKUP STACK 1   ##
############################

bb(m=Move.To(MaterialStack.STACK1.x + 35, 
             MaterialStack.STACK1.y - 350, 
             'f', 2000, 2000, 15, 20),
  s=[Servo.FrontVacuumLift(VacuumLift.UP),
     Servo.CenterLift(CenterLift.DOWN),
     Servo.CenterSwing(CenterSwing.DOWN),
     Servo.FrontVacuum(Vacuum.DOWN),
     Servo.FrontGripLift(FrontGripLift.DOWN)])

bb(m=Move.RotateTo(1.57, 15, 3))
bb(task_steps=pickup_front_full_stack())

############################
##  PICKUP STACK 8   ##
############################

bb(m=Move.Spline([MaterialStack.STACK8.x - 225],
                 [MaterialStack.STACK8.y - 15],
                 [3.14],
                 450, #400
                 'r'))

# bb(m=Move.RotateTo(0, 15, 5))
bb(task_steps=pickup_back_full_stack())
# bb(s=[Servo.BackLift(BackGripLift.UP)])


####################################
##  DROP STACK 1 TO AREA BLUE 2   ##
####################################

bb(m=Move.Spline([Area.BLUE_2.x],
                 [Area.BLUE_2.y + 550],
                 [-1.57],
                 450, # 400
                 'f'),
    s=[Servo.BackLift(BackGripLift.UP)],
    task_steps=two_level())
  
bb(s=[Servo.BackLift(BackGripLift.DOWN, 30)])

bb(m=Move.Distance(390, 500, 500),
  s=[Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN),
      Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN)])

bb(task_steps=drop_two_level())

bb(m=Move.RotateTo(1.57, 15, 10),
  s=[Servo.FrontVacuumLift(VacuumLift.UP),
     Servo.CenterLift(CenterLift.DOWN),
     Servo.CenterSwing(CenterSwing.DOWN),
     Servo.FrontVacuum(Vacuum.DOWN),
     Servo.FrontGripLift(FrontGripLift.DOWN)])

bb(task_steps=pickup_front_full_stack())

bb(m=Move.RotateTo(-1.57, 3, 3),
  task_steps=two_level())

bb(m=Move.Distance(330, 500, 500))

bb(task_steps=drop_two_level())

bb(m=Move.To(Area.BLUE_HOME.x, Area.BLUE_HOME.y-250, 'r', 2000, 2000, 15, 20))


# bb(m=Move.RotateTo( ))

# bb(m=Move.Spline([],
#                 [],
#                 [],
#                 500, 
#                 'f'))












