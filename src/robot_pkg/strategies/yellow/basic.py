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

basic(task_steps=pickup_front_full_stack(280)) # 270

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
basic(task_steps=drop_one_level(backout_distance=-175, p=-4))
basic(m=Move.RotateTo(0, 15, 15),
      task_steps=close_back())
basic(task_steps=pickup_back_full_stack(-245))

#############################################
## MOVE TO YELLOW AREA 4 AND LIFT ONE ON TWO ##
#############################################

basic(m=Move.To(MaterialStack.STACK5.x + 10,
                MaterialStack.STACK5.y + 200,
                'f', 1000, 800, 10, 5),
      s=[Servo.BackLift(BackGripLift.UP)])   # HOVER)])
basic(m=Move.RotateTo(-1.57, 15, 10))  # 5, 5))

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
                Area.YELLOW_2.y,
                'r', 1000, 500, 10, 5),
      task_steps=two_level())

basic(m=Move.RotateTo(1.57, 10, 5),
      s=[Servo.BackLift(BackGripLift.DOWN)])
basic(task_steps=open_back())

basic(m=Move.Distance(250, 500, 300))

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

basic(task_steps=pickup_back_full_stack())

basic(m=Move.Distance(250, 1000, 500),
      s=[Servo.BackLift(BackGripLift.UP2)])

##########################
##  MOVE TO YELLOW AREA 2 ##
##########################

# TODO brzi spline i slaganje tek kada stigne zbog obima
# PAZI -J
basic(m=Move.Spline([Area.YELLOW_2.x],
                    [Area.YELLOW_2.y + 550],
                    [-1.57],
                    500,
                    'f'),
      task_steps=two_level())

#####################################################
## LEAVE STACK 3 FROM BACK IN FRONT OF YELLOW AREA 2 ##
## DROP TWO LEVELS IN YELLOW AREA 2                  ##
#####################################################

basic(s=[Servo.BackLift(BackGripLift.DOWN)])

basic(m=Move.Distance(390, 500, 500),
      task_steps=open_back())

basic(task_steps=drop_two_level())

#######################################
## PICK-UP STACK 3 WHERE IT WAS LEFT ##
## AND BUILD TWO LEVELS              ##
#######################################

basic(m=Move.RotateTo(1.57, 15, 10), # PAZI
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(300))

basic(task_steps=two_level())

###############################################
## PICK-UP LOWER WITH BACK                   ##
## LIFT UPPER ON CONSTRUCTION IN YELLOW AREA 2 ##
###############################################

basic(task_steps=drop_one_level(p=-4))

basic(m=Move.RotateTo(-1.57, 15, 10), # PAZI
      task_steps=init_back_servos())

basic(task_steps=pickup_back_full_stack())

basic(task_steps=lift_one_on_two(forward_distance=380+205, backout_distance=-200))

##############################################
## LEAVE ONE LEVEL FROM BACK IN BLUE AREA 2 ##
##############################################

basic(m=Move.RotateTo(1.57, 10, 5),
      task_steps=init_front_servos())

basic(m=Move.Distance(-100, 500, 300),
      task_steps=open_back(),
      p=Points.LEVEL1)

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

###################################
## SEPARATE STACK IN YELLOW AREA 3 ##
###################################

basic(m=Move.Spline([Area.YELLOW_3.x - 550],
                    [Area.YELLOW_3.y],
                    [0],
                    400,
                    'f'),
      task_steps=two_level())
basic(s=[Servo.BackLift(BackGripLift.DOWN)])

basic(m=Move.Distance(390, 500, 500),
      task_steps=open_back())

basic(task_steps=drop_two_level())

basic(m=Move.RotateTo(-3.14, 15, 10),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(300))

basic(task_steps=two_level())

basic(task_steps=drop_one_level(p=-4))

basic(m=Move.RotateTo(0, 15, 10),
      task_steps=init_back_servos())

basic(task_steps=pickup_back_full_stack())

basic(task_steps=lift_one_on_two(forward_distance=380+200))

##############################################
## LEAVE ONE LEVEL FROM BACK IN YELLOW AREA 3 ##
##############################################

basic(m=Move.RotateTo(3.14, 10, 5))

basic(m=Move.Distance(-100, 500, 300),
      task_steps=open_back(),
      p=Points.LEVEL1)

basic(m=Move.Distance(100, 500, 300),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(-1.57, 15, 10))

################################
## PICK-UP STACK 7 WITH FRONT ##
################################

basic(m=Move.Spline([MaterialStack.STACK7.x - 225],
                    [MaterialStack.STACK7.y],
                    [0],
                    400,
                    'f'),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack())

basic(m=Move.Distance(-150, 500, 500))

basic(m=Move.RotateTo(3.14, 10, 5))

####################################
## LIFT STACK 7 ON ONE LEVEL IN YELLOW AREA 3 ##
####################################

basic(m=Move.Spline([Area.YELLOW_3.x - 350],
                    [Area.YELLOW_3.y],
                    [0],
                    400,
                    'f'),
      task_steps=two_level())

basic(task_steps=lift_two_on_one())

#####################
## PICK-UP STACK 8 ##
#####################

basic(m=Move.RotateTo(1.57, 15, 15))
basic(m=Move.Spline([MaterialStack.STACK8.x - 175],
                    [MaterialStack.STACK8.y],
                    [0],
                    400, 'f'),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack())

basic(m=Move.Distance(-100, 500, 500))

basic(m=Move.RotateTo(3.14, 10, 5))

basic(m=Move.Spline([Area.YELLOW_3.x - 450],
                    [Area.YELLOW_3.y],
                    [0],
                    400,
                    'f'),
      task_steps=two_level())

basic(task_steps=drop_two_level())

basic(m=Move.RotateTo(3.14, 15, 15),
      c=[Condition.InPosition(100),])


####################################
## NO STACK 9 ALTERNATIVE - ID=5 ##
####################################

basic(ID=5)

################################################
## LEAVE LOWER LEVEL FROM BACK IN YELLOW AREA 2 ##
################################################

basic(m=Move.To(Area.YELLOW_2.x,
                Area.YELLOW_2.y,
                'r', 1000, 500, 10, 5))

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
                    [MaterialStack.STACK3.y - 30],
                    [0],
                    450,
                    'r'))

basic(task_steps=pickup_back_full_stack())#id=6))
# TODO nema STACK 8 - ostaviti dvospratnicu od STACK 1 u BLUE AREA 2

basic(m=Move.Distance(250, 1000, 500),
      s=[Servo.BackLift(BackGripLift.UP2)])

##########################
##  MOVE TO YELLOW AREA 2 ##
##########################

# TODO brzi spline i slaganje tek kada stigne zbog obima
basic(m=Move.Spline([Area.YELLOW_2.x],
                    [Area.YELLOW_2.y + 550],
                    [-1.57],
                    500,
                    'f'),
      task_steps=two_level())

#####################################################
## LEAVE STACK 3 FROM BACK IN FRONT OF YELLOW AREA 2 ##
## DROP TWO LEVELS IN YELLOW AREA 2                  ##
#####################################################

basic(s=[Servo.BackLift(BackGripLift.DOWN)])

basic(m=Move.Distance(390, 500, 500),
      task_steps=open_back())

basic(task_steps=drop_two_level())

#######################################
## PICK-UP STACK 3 WHERE IT WAS LEFT ##
## AND BUILD TWO LEVELS              ##
#######################################

basic(m=Move.RotateTo(1.57, 15, 10),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(300))

basic(task_steps=two_level())

###############################################
## PICK-UP LOWER WITH BACK                   ##
## LIFT UPPER ON CONSTRUCTION IN YELLOW AREA 2 ##
###############################################

basic(task_steps=drop_one_level(p=-4))

basic(m=Move.RotateTo(-1.57, 15, 10),
      task_steps=init_back_servos())

basic(task_steps=pickup_back_full_stack())

basic(task_steps=lift_one_on_two(forward_distance=380+200))

##############################################
## LEAVE ONE LEVEL FROM BACK IN YELLOW AREA 2 ##
##############################################

basic(m=Move.RotateTo(1.57, 10, 5))

basic(m=Move.Distance(-100, 500, 300),
      task_steps=open_back(),
      p=Points.LEVEL1)

basic(m=Move.Distance(100, 500, 300),
      c=[Condition.InPosition(100),])

# TODO dodati deo gde kupi jos jedan stack od negde


##########
## HOME ##
##########
basic(ID=100,
      m=Move.Distance(150, 1000, 1000),
      task_steps=open_all())

basic(m=Move.To(Area.YELLOW_HOME.x, Area.YELLOW_HOME.y - 450, 'f', 1100, 1500, 15, 15),
      task_steps=init_all_servos())

# Wait for 99s to enter area
basic(c=[Condition.MatchTime(101, 99)])


basic(ID=101,
      m=Move.To(Area.YELLOW_HOME.x, Area.YELLOW_HOME.y -
                250, 'f', 1100, 1500, 15, 15),
      p=10)
