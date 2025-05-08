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
        sima1_coor=[Position(125, -1605, 0, 0),
                    Position(1000, -1300, 0, 50),
                    Position(1800, -1400, 0, 50)],
        sima2_coor=[Position(125, -1724, 0, 0),
                    Position(300, -1724, 0, 50),
                    Position(900, -1400, 0, 50),
                    Position(1300, -1360, 0, 50)],
        sima3_coor=[Position(125, -1815, 0, 0),
                    Position(500, -1815, 0, 50),
                    Position(900, -1500, 0, 50)],
        sima4_coor=[Position(125, -1915, 0, 0),
                    Position(1300, -1915, 0, 20),
                    Position(1300, -1415, 0, 4)])
)

basic(task_steps=init_position(
    Area.YELLOW_2.x + 77.5,
    Area.YELLOW_2.y + 48,
    0.0,
    'middle'))

###################################
## LEAVING BANNER IN YELLOW AREA 2 ##
###################################

basic(ID=1,
      task_steps=leave_banner())

#####################
## PICK-UP STACK 5 ##
#####################

basic(m=Move.Spline([MaterialStack.STACK5.x - 10],
                    [MaterialStack.STACK5.y + 250],
                    [-1.57],
                    550,
                    'f'),
      task_steps=init_front_servos(),
      c=[Condition.Detection(2, attempts=0),])

basic(m=Move.RotateTo(-1.57, 10, 10))

basic(task_steps=pickup_front_full_stack(280))  # 270

#####################################
## BUILD TWO LEVELS IN YELLOW AREA 4 ##
#####################################

basic(task_steps=two_level())
basic(task_steps=drop_two_level())

#####################
## PICK-UP STACK 4 ##
#####################

basic(m=Move.To(MaterialStack.STACK4.x + 290,
                MaterialStack.STACK4.y - 15,
                'f', 1500, 1500, 15, 15),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(3.14, 15, 15))

basic(task_steps=pickup_front_full_stack(200))

####################################
## SEPARATE STACK 4               ##
## PICK-UP LOWER LEVEL WITH BACK  ##
####################################

basic(m=Move.Distance(-350, 800, 500),
      task_steps=two_level())

basic(task_steps=separate_two_level())

#############################################
## MOVE TO YELLOW AREA 4 AND LIFT ONE ON TWO ##
#############################################

basic(m=Move.To(MaterialStack.STACK5.x + 10,
                MaterialStack.STACK5.y + 200,
                'f', 1000, 800, 10, 5),
      s=[Servo.BackLift(BackGripLift.UP)])
basic(m=Move.RotateTo(-1.57, 15, 10))

basic(task_steps=lift_one_on_two(forward_distance=245))

######################
## PICK-UP STACK 9 ##
######################

basic(m=Move.To(MaterialStack.STACK9.x,
                MaterialStack.STACK9.y - 350,
                'f', 1500, 1000, 15, 10),  # 5, 3),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(1.57, 15, 10))  # 5, 5))

basic(task_steps=(pickup_front_full_stack(330, ID=5)))

################################################
## LEAVE LOWER LEVEL FROM BACK IN YELLOW AREA 2 ##
################################################

basic(m=Move.To(Area.YELLOW_2.x,
                Area.YELLOW_2.y + 20,
                'r', 1000, 500, 10, 5),
      task_steps=two_level())

basic(task_steps=drop_back_one_level(rotation=1.57))

###################################
## BUILD TWO LEVELS ON TOP OF IT ##
###################################

basic(m=Move.RotateTo(-1.57, 5, 5))

basic(task_steps=lift_two_on_one(250))

basic(task_steps=init_front_servos())

#####################
##  PICKUP STACK 2 ##
#####################

basic(m=Move.To(MaterialStack.STACK2.x-10,
                MaterialStack.STACK2.y - 365,
                'f', 1500, 1500, 15, 15),
      task_steps=close_front())

basic(m=Move.RotateTo(1.57, 15, 10))
basic(task_steps=pickup_front_full_stack())

################################
##  PICKUP STACK 3  WITH BACK ##
################################

basic(m=Move.Spline([MaterialStack.STACK3.x + 215],
                    [MaterialStack.STACK3.y - 15],
                    [0.0],
                    450,
                    'r'))

basic(task_steps=pickup_back_full_stack(-180))

basic(m=Move.Distance(250, 1000, 500),
      s=[Servo.BackLift(BackGripLift.UP2)])

##########################
##  MOVE TO YELLOW AREA 2 ##
##########################

# TODO brzi spline i slaganje tek kada stigne zbog obima
# PAZI -J
basic(m=Move.Spline([Area.YELLOW_2.x],
                    [Area.YELLOW_2.y + 570],
                    [-1.57],
                    500,
                    'f'),
      task_steps=two_level())  # TODO testirati slaganje u detekciji

#####################################################
## LEAVE STACK 3 FROM BACK IN FRONT OF YELLOW AREA 2 ##
## DROP TWO LEVELS IN YELLOW AREA 2                  ##
#####################################################

basic(task_steps=drop_back_one_level(forward_distance=390, p=-4))

basic(task_steps=drop_two_level())

#######################################
## PICK-UP STACK 3 WHERE IT WAS LEFT ##
## AND BUILD TWO LEVELS              ##
#######################################

basic(m=Move.RotateTo(1.57, 15, 10),  # PAZI
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(300))

basic(task_steps=two_level())

###############################################
## PICK-UP LOWER WITH BACK                   ##
## LIFT UPPER ON CONSTRUCTION IN YELLOW AREA 2 ##
###############################################

basic(task_steps=drop_one_level(p=-4))

basic(m=Move.RotateTo(-1.57, 15, 10),  # PAZI
      task_steps=init_back_servos())

basic(task_steps=pickup_back_full_stack())

basic(task_steps=lift_one_on_two(forward_distance=380+205, backout_distance=-200))

##############################################
## LEAVE ONE LEVEL FROM BACK IN BLUE AREA 2 ##
##############################################

basic(task_steps=drop_back_one_level(rotation=1.57, forward_distance=-100))

basic(m=Move.Distance(100, 500, 300),
      c=[Condition.InPosition(100),])

########################
## END OF MAIN BRANCH ##
########################


#############################################
## DETECTION ON STACK 5 ALTERNATIVE - ID 2 ##
#############################################

basic(ID=2)

######################
## PICK-UP STACK 10 ##
######################

basic(m=Move.To(MaterialStack.STACK10.x - 10,
                MaterialStack.STACK10.y - 350,
                'f', 1500, 1000, 15, 10),  # 5, 3),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(1.57, 15, 10))  # 5, 5))

basic(task_steps=(pickup_front_full_stack(300)))

###############################
## PICK-UP STACK 6 WITH BACK ##
###############################

basic(m=Move.To(MaterialStack.STACK6.x,
                MaterialStack.STACK6.y + 250,
                'r', 1500, 1000, 15, 10),
      task_steps=init_back_servos())

basic(m=Move.RotateTo(1.57, 15, 10))

basic(task_steps=pickup_back_full_stack())

########################################
## LEAVE STACK 6 IN FRONT OF YELLOW AREA 3 ##
########################################

basic(m=Move.Spline([Area.YELLOW_3.x - 450],
                    [Area.YELLOW_3.y],
                    [0],
                    400,
                    'f'),
      task_steps=two_level())

basic(task_steps=drop_back_one_level(forward_distance=390, p=-4))

########################################
## BUILD THREE LEVELS FROM STACK 10 AND STACK 6 IN YELLOW AREA 3 ##
########################################

basic(task_steps=drop_two_level())

basic(m=Move.RotateTo(3.14, 15, 10),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(325))

basic(task_steps=two_level())

basic(task_steps=drop_one_level(p=-4))

basic(m=Move.RotateTo(0, 15, 10),
      task_steps=init_back_servos())

basic(task_steps=pickup_back_full_stack())

basic(task_steps=lift_one_on_two(forward_distance=380+200))

basic(task_steps=init_front_servos())

##############################################
## LEAVE ONE LEVEL FROM BACK IN YELLOW AREA 3 ##
##############################################

basic(task_steps=drop_back_one_level(rotation=3.14, forward_distance=-100))

basic(m=Move.Distance(300, 500, 300),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(-1.57, 15, 10))

################################
## PICK-UP STACK 7 WITH FRONT ##
################################

basic(m=Move.Spline([MaterialStack.STACK7.x - 225],
                    [MaterialStack.STACK7.y - 10],
                    [0],
                    400,
                    'f'),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(100))

basic(m=Move.Distance(-150, 500, 500))

# basic(m=Move.RotateTo(3.14, 10, 5))

####################################
## LIFT STACK 7 ON ONE LEVEL IN YELLOW AREA 3 ##
####################################

basic(m=Move.Spline([Area.YELLOW_3.x - 650],
                    [Area.YELLOW_3.y],
                    [0],
                    300,
                    'r'),
      task_steps=two_level())

basic(task_steps=lift_two_on_one(forward_distance=440))

basic(m=Move.RotateTo(3.14, 15, 15),
      task_steps=init_front_servos())

#####################
## PICK-UP STACK 8 ##
#####################

# basic(m=Move.RotateTo(1.57, 15, 15))
# basic(m=Move.Spline([MaterialStack.STACK8.x - 175],
#                     [MaterialStack.STACK8.y],
#                     [0],
#                     400, 'f'),
#       task_steps=init_front_servos())

# basic(task_steps=pickup_front_full_stack(ID=6))

# basic(m=Move.Distance(-150, 500, 500))

# basic(m=Move.RotateTo(0, 10, 5))

#####################
## PICK-UP STACK 2 ##
#####################

# TODO proveriti da li je oko 85s
basic(m=Move.Spline([1500, MaterialStack.STACK2.x-10],
                    [1250, MaterialStack.STACK2.y - 400],
                    [3.14, 1.57],
                    800,
                    'f'),
      task_steps=close_front())

basic(task_steps=pickup_front_full_stack(300))

#####################
## LEAVE IN YELLOW AREA 2 ##
#####################

basic(m=Move.Spline([Area.YELLOW_2.x],
                    [Area.YELLOW_2.y + 50],
                    [1.57],
                    400,
                    'r'),
      s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
         Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN)],
      task_steps=two_level())

# basic(task_steps=drop_back_one_level(forward_distance=390, p=-4))
basic(c=[Condition.BackSensors(10)])
basic(p=4)

# No point if STACK 10 wasn't there
basic(ID=10)

basic(m=Move.Distance(300, 500, 300))
basic(m=Move.RotateTo(-1.57, 10, 5))
basic(task_steps=drop_two_level(-200))

basic(m=Move.RotateTo(3.14, 10, 5))

#################################
## PICK-UP STACK 5             ##
#################################

basic(m=Move.Spline([MaterialStack.STACK5.x - 25],
                    [MaterialStack.STACK5.y + 250],
                    [-1.57],
                    400,
                    'f'),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(300, ID=11))
basic(task_steps=two_level())
basic(task_steps=drop_two_level(-250))

basic(ID=11)

basic(m=Move.Distance(-100, 500, 300),
      c=[Condition.InPosition(100),])

####################################
## NO STACK 9 ALTERNATIVE - ID=5 ##
####################################

basic(ID=5)

################################################
## LEAVE LOWER LEVEL FROM BACK IN YELLOW AREA 2 ##
################################################

basic(m=Move.To(Area.YELLOW_2.x,
                Area.YELLOW_2.y + 20,
                'r', 1000, 1000, 15, 10))

basic(m=Move.RotateTo(1.57, 10, 5),
      s=[Servo.BackLift(BackGripLift.DOWN)])
basic(task_steps=open_back())

basic(m=Move.Distance(250, 500, 300))

#####################
##  PICKUP STACK 2 ##
#####################

basic(m=Move.To(MaterialStack.STACK2.x - 15,
                MaterialStack.STACK2.y - 365,
                'f', 1500, 1500, 15, 15),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(1.57, 15, 10))
basic(task_steps=pickup_front_full_stack())

################################
##  PICKUP STACK 3 WITH BACK ##
################################

basic(m=Move.Spline([MaterialStack.STACK3.x + 215],
                    [MaterialStack.STACK3.y - 15],
                    [0],
                    450,
                    'r'))

basic(task_steps=pickup_back_full_stack(-160))  # id=6))
# TODO nema STACK 3 - ostaviti dvospratnicu od STACK 1 u BLUE AREA 2

basic(m=Move.Distance(250, 1000, 500),
      s=[Servo.BackLift(BackGripLift.UP2)])

##########################
##  MOVE TO YELLOW AREA 2 ##
##########################

# TODO brzi spline i slaganje tek kada stigne zbog obima
basic(m=Move.Spline([Area.YELLOW_2.x],
                    [Area.YELLOW_2.y + 450],
                    [-1.57],
                    500,
                    'f'),
      task_steps=two_level())

#####################################################
## LEAVE STACK 3 FROM BACK IN FRONT OF YELLOW AREA 2 ##
## DROP TWO LEVELS IN YELLOW AREA 2                  ##
#####################################################

basic(s=[Servo.BackLift(BackGripLift.DOWN)])

basic(task_steps=open_back())

basic(task_steps=lift_two_on_one(forward_distance=420, back_distance=-200))

#######################################
## PICK-UP STACK 3 WHERE IT WAS LEFT ##
## AND BUILD TWO LEVELS              ##
#######################################

basic(m=Move.RotateTo(1.57, 15, 10),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(250))

basic(m=Move.RotateTo(-1.57, 10, 5),
      task_steps=two_level())

basic(m=Move.Distance(300, 500, 500))

basic(task_steps=drop_two_level())

##############################################
## PIKC-UP STACK 7 ##
##############################################

# Proveriti vreme
basic(m=Move.To(MaterialStack.STACK7.x-300, MaterialStack.STACK7.y-15, 'f', 1500, 1500, 15, 10),
      task_steps=(init_front_servos()))

basic(m=Move.RotateTo(0, 15, 15))

basic(task_steps=pickup_front_full_stack(forward_distance=200, ID=7))

basic(m=Move.Distance(-150, 500, 500))

basic(m=Move.RotateTo(1.57, 10, 5))

basic(m=Move.Distance(400, 500, 500),
      task_steps=two_level())

basic(m=Move.RotateTo(0, 10, 5))

basic(task_steps=drop_two_level())

basic(ID=7)

basic(m=Move.Distance(-100, 500, 300),
      c=[Condition.InPosition(100),])

# TODO dodati deo gde kupi jos jedan stack od negde


##########
## HOME ##
##########
basic(ID=100,
      m=Move.Distance(150, 1000, 1000),
      task_steps=open_all())

basic(m=Move.To(Area.YELLOW_HOME.x, Area.YELLOW_HOME.y - 550, 'f', 1100, 1500, 15, 15),
      task_steps=init_all_servos())

# Wait for 99s to enter area
basic(c=[Condition.MatchTime(101, 99)])


basic(ID=101,
      m=Move.To(Area.YELLOW_HOME.x, Area.YELLOW_HOME.y -
                250, 'f', 1100, 1500, 15, 15),
      p=10)
