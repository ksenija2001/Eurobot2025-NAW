from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *

########################
## YELLOW LOWER PASSIVE ##
########################

basic = Strategy(color=Color.YELLOW, square=Square.LOWER, mood=Mood.PASSIVE)

#################
## PREPARATION ##
#################

basic(
    task_steps=sima_coordinates(
        sima1_coor=[Position(1000, 1000, 1.57, 100),
                    Position(1500, 1500, 0, 100),
                    Position(900, 900.5, 0, 200)],
        sima2_coor=[],
        sima3_coor=[],
        sima4_coor=[Position(125, -400, 0, 0),
                    Position(1000, -600, 0, 35),
                    Position(1800, -550, 0, 35)])
)

basic(task_steps=init_position(
    Area.BLUE_2.x + 77.5,
    Area.BLUE_2.y + 48,
    0.0,
    'middle'))

###################################
## LEAVING BANNER IN BLUE AREA 2 ##
###################################

basic(ID=1,
      task_steps=leave_banner())

#####################
## PICK-UP STACK 6 ##
#####################

basic(m=Move.Spline([MaterialStack.STACK6.x + 10],
                    [MaterialStack.STACK6.y + 250],
                    [-1.57],
                    500,
                    'f'),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(-1.57, 10, 10))

basic(task_steps=pickup_front_full_stack(270))

#####################################
## BUILD TWO LEVELS IN BLUE AREA 4 ##
#####################################

basic(task_steps=two_level())
basic(task_steps=drop_two_level())

#####################
## PICK-UP STACK 7 ##
#####################

basic(m=Move.To(MaterialStack.STACK7.x - 290,
                MaterialStack.STACK7.y - 15,
                'f', 1500, 1500, 15, 15),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(0, 15, 15))

basic(task_steps=pickup_front_full_stack(200))

####################################
## SEPARATE STACK 7               ##
## PICK-UP LOWER LEVEL WITH BACK  ##
####################################

basic(m=Move.Distance(-350, 1500, 500),
      task_steps=two_level())
basic(task_steps=drop_one_level(p=-4))
basic(m=Move.RotateTo(3.14, 15, 15))
basic(task_steps=pickup_back_full_stack(-220))

#############################################
## MOVE TO BLUE AREA 4 AND LIFT ONE ON TWO ##
#############################################

basic(m=Move.To(MaterialStack.STACK6.x + 10,
                MaterialStack.STACK6.y + 200,
                'f', 1000, 800, 10, 5),
      s=[Servo.BackLift(BackGripLift.UP)])   # HOVER)])
basic(m=Move.RotateTo(-1.57, 15, 10))  # 5, 5))

basic(task_steps=lift_one_on_two(forward_distance=250))

######################
## PICK-UP STACK 10 ##
######################

basic(m=Move.To(MaterialStack.STACK10.x + 10,
                MaterialStack.STACK10.y - 350,
                'f', 1500, 1000, 15, 10),  # 5, 3),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(1.57, 15, 10))  # 5, 5))

basic(task_steps=(pickup_front_full_stack(300)))

################################################
## LEAVE LOWER LEVEL FROM BACK IN BLUE AREA 2 ##
################################################

basic(m=Move.To(Area.BLUE_2.x,
                Area.BLUE_2.y,
                'r', 1000, 500, 10, 5),
      task_steps=two_level())

basic(m=Move.RotateTo(1.57, 10, 5),
      s=[Servo.BackLift(BackGripLift.DOWN)])

basic(m=Move.Distance(250, 500, 300),
      task_steps=open_back())

###################################
## BUILD TWO LEVELS ON TOP OF IT ##
###################################

basic(m=Move.RotateTo(-1.57, 10, 5))

basic(task_steps=lift_two_on_one(250))

#####################
##  PICKUP STACK 1 ##
#####################

basic(m=Move.To(MaterialStack.STACK1.x + 15,
                MaterialStack.STACK1.y - 365,
                'f', 1500, 1500, 15, 15),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(1.57, 15, 10))
basic(task_steps=pickup_front_full_stack())

################################
##  PICKUP STACK 8  WITH BACK ##
################################

basic(m=Move.Spline([MaterialStack.STACK8.x - 215],
                    [MaterialStack.STACK8.y - 30],
                    [3.14],
                    450,
                    'r'))

basic(task_steps=pickup_back_full_stack())

basic(m=Move.Distance(250, 1000, 500),
      s=[Servo.BackLift(BackGripLift.UP)])

##########################
##  MOVE TO BLUE AREA 2 ##
##########################

# TODO brzi spline i slaganje tek kada stigne zbog obima
basic(m=Move.Spline([Area.BLUE_2.x],
                    [Area.BLUE_2.y + 550],
                    [-1.57],
                    500,
                    'f'),
      task_steps=two_level())

#####################################################
## LEAVE STACK 8 FROM BACK IN FRONT OF BLUE AREA 2 ##
## DROP TWO LEVELS IN BLUE AREA 2                  ##
#####################################################

basic(s=[Servo.BackLift(BackGripLift.DOWN, 30)])

basic(m=Move.Distance(390, 500, 500),
      task_steps=open_back())

basic(task_steps=drop_two_level())

#######################################
## PICK-UP STACK 8 WHERE IT WAS LEFT ##
## AND BUILD TWO LEVELS              ##
#######################################

basic(m=Move.RotateTo(1.57, 15, 10),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(300))

basic(task_steps=two_level())

###############################################
## PICK-UP LOWER WITH BACK                   ##
## LIFT UPPER ON CONSTRUCTION IN BLUE AREA 2 ##
###############################################

basic(task_steps=drop_one_level())

basic(m=Move.RotateTo(-1.57, 15, 10),
      task_steps=init_back_servos())

basic(task_steps=pickup_back_full_stack())

basic(task_steps=lift_one_on_two(forward_distance=380+200))

##############################################
## LEAVE ONE LEVEL FROM BACK IN BLUE AREA 2 ##
##############################################

basic(m=Move.RotateTo(1.57, 10, 5))

basic(m=Move.Distance(-100, 500, 300),
      task_steps=open_back(),
      p=Points.LEVEL1)

basic(m=Move.Distance(100, 500, 300))

##########
## HOME ##
##########

basic(ID=100,
      task_steps=open_all())

basic(m=Move.Spline([Area.BLUE_HOME.x],
                    [Area.BLUE_HOME.y - 250],
                    [1.57],
                    800,
                    'f'),
      task_steps=init_all_servos(),
      c=[Condition.InPosition(102)])


# KEEP AT BOTTOM OF STARTEGY - ensures points for home are given when movement is done
basic(ID=102, p=10)
