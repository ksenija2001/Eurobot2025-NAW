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

bb = Strategy(color = Color.BLUE, square = Square.LOWER, mood = Mood.PASSIVE)

# bb(sima_id=4, 
#   sima=[Position(125, -400, 0, 0), 
#         Position(1000, -600, 0, 35), 
#         Position(1800, -550, 0, 35)])
# # bb(sima_id=1, 
# #   sima=[Position(1000, 1000, 1.57, 100), Position(1500, 1500, 0, 100), Position(900, 900.5, 0, 200)])

bb(task_steps=init_position(
                            Area.BLUE_2.x + 77.5, 
                            Area.BLUE_2.y + 48, 
                            0.0, 
                            'middle'))

####################
## LEAVING BANNER ##
####################
# bb(m=Move.Distance(-100, 1000, 500))
# bb(m=Move.Distance(100, 1000, 500))

##############################
## PICK-UP and DROP STACK 6 ##
##############################
bb(ID=1, 
    m=Move.Spline([MaterialStack.STACK6.x + 10],
                 [MaterialStack.STACK6.y + 250],
                 [-1.57],
                 500,
                 'f'),
    task_steps=init_front_servos())
    # s=[Servo.CenterSwing(CenterSwing.DOWN),
    #   Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
    #   Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN)])

bb(m=Move.RotateTo(-1.57, 5, 5))

bb(task_steps=pickup_front_full_stack(270))
bb(task_steps=two_level())
bb(task_steps=drop_two_level())

#####################
## PICK-UP STACK 7 ##
#####################

bb(m=Move.To(MaterialStack.STACK7.x - 290, 
             MaterialStack.STACK7.y - 15, 
             'f', 1500, 1500, 15, 15),
    task_steps=init_front_servos())

bb(m=Move.RotateTo(0, 15, 15))

bb(task_steps=pickup_front_full_stack(200))
bb(m=Move.Distance(-350, 1500, 500),
    task_steps=two_level())
bb(task_steps=drop_one_level())
bb(m=Move.RotateTo(3.14, 15, 15))
bb(task_steps=pickup_back_full_stack())


# bb(s=[Servo.BackLift(BackGripLift.UP)])

######################
## PICK-UP STACK 10 ##
######################

# bb(m=Move.Spline([MaterialStack.STACK10.x + 10],
#                  [MaterialStack.STACK10.y - 260],
#                  [1.57],
#                  500,
#                  'f'),
#   s=[Servo.BackLift(BackGripLift.UP),
#      Servo.FrontVacuumLift(VacuumLift.UP),
#      Servo.FrontVacuum(Vacuum.DOWN),
#      Servo.CenterLift(CenterLift.DOWN),
#      Servo.CenterSwing(CenterSwing.DOWN)
#      ])

# bb(task_steps=pickup_front_full_stack(30))

############################
## SORT and DROP STACK 10 ##
############################


# bb(m=Move.To(Area.BLUE_2.x, Area.BLUE_2.y + 50, 'f', 1000, 1000, 10, 3),
#   task_steps=two_level())

# bb(m=Move.RotateTo(-1.57, 15, 10))

# bb(task_steps=drop_one_level(-150))

#########################################
##  DROPPING STACK 7 FROM BACK GRIPPER ##
#########################################

# bb(m=Move.RotateTo(0, 10, 5),
#   s=[Servo.BackLift(BackGripLift.DOWN, 30)])

# bb(s=[Servo.BackLift(BackGripLift.DOWN, 30)])
# bb(m=Move.Distance(100, 1500, 1500),
#   s=[Servo.BackSideGrip(Gripper.OPEN),
#      Servo.BackCenterGrip(Gripper.OPEN)])

#########################################
## MOVE TO STACK 6 AND LIFT ONE ON TWO ##
#########################################

bb(m=Move.To(MaterialStack.STACK6.x + 10, MaterialStack.STACK6.y + 200, 'f', 1000, 800, 10, 5),
    s=[Servo.BackLift(BackGripLift.HOVER)])
bb(m=Move.RotateTo(-1.57, 5, 5))
# bb(m=Move.To(MaterialStack.STACK6.x+25, MaterialStack.STACK6.y+200, 'f', 1500, 1500, 15, 5))
# bb(m=Move.RotateTo(-1.57, 15, 5))

bb(task_steps=lift_one_on_two(forward_distance=250))


bb(m=Move.To(MaterialStack.STACK10.x+10, MaterialStack.STACK10.y-350, 'f', 1500, 1000, 5, 3),
  task_steps=init_front_servos())
     
bb(m=Move.RotateTo(1.57, 5, 5))
bb(task_steps=(pickup_front_full_stack(300)))

bb(m=Move.To(Area.BLUE_2.x, Area.BLUE_2.y, 'r', 1000, 500, 10, 5), # changed from r
  task_steps=two_level())

bb(m=Move.RotateTo(1.57, 5, 5))

bb(m=Move.Distance(250, 300, 300),
  s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
     Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN)])

bb(m=Move.RotateTo(-1.57, 15, 10))

bb(task_steps=lift_two_on_one(250))

############################
##  PICKUP STACK 1   ##
############################

bb(m=Move.To(MaterialStack.STACK1.x + 15, 
             MaterialStack.STACK1.y - 350, 
             'f', 1500, 1500, 15, 15),
  task_steps=init_front_servos())

bb(m=Move.RotateTo(1.57, 15, 10))
bb(task_steps=pickup_front_full_stack())

############################
##  PICKUP STACK 8   ##
############################

bb(m=Move.Spline([MaterialStack.STACK8.x - 225],
                 [MaterialStack.STACK8.y - 15],
                 [3.14],
                 450, #400
                 'r'))

bb(task_steps=pickup_back_full_stack())

bb(m=Move.Distance(250, 1000, 500),
    s=[Servo.BackLift(BackGripLift.UP)])

####################################
##  DROP STACK 1 TO AREA BLUE 2   ##
####################################

bb(m=Move.Spline([Area.BLUE_2.x],
                 [Area.BLUE_2.y + 550],
                 [-1.57],
                 550, #450, # 400
                 'f'),
    task_steps=two_level())
  
bb(s=[Servo.BackLift(BackGripLift.DOWN, 30)])

bb(m=Move.Distance(390, 500, 500),
  s=[Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN),
      Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN)])

bb(task_steps=drop_two_level())

bb(m=Move.RotateTo(1.57, 15, 10),
    task_steps=init_front_servos())

bb(task_steps=pickup_front_full_stack())

bb(task_steps=two_level())
bb(task_steps=drop_one_level())

bb(m=Move.RotateTo(-1.57, 15, 10))#,
  # task_steps=two_level())

bb(task_steps=lift_one_on_two(forward_distance=380))

bb(task_steps=pickup_back_full_stack())

bb(m=Move.RotateTo(-1.57, 10, 5))

bb(m=Move.Distance(200, 500, 500),
    task_steps=open_back())

# bb(m=Move.Distance(330, 500, 500))

# bb(task_steps=drop_two_level())

# bb(m=Move.To(Area.BLUE_HOME.x, Area.BLUE_HOME.y-250, 'r', 2000, 2000, 15, 20))

##########
## HOME ##
##########

bb(ID=100,
    m=Move.Spline([Area.BLUE_HOME.x], 
                [Area.BLUE_HOME.y-250], 
                [2.35],
                800,
                'f'))












