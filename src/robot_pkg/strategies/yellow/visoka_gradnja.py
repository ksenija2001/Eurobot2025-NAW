from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *

###########################
## YELLOW UPPER AGGRESSIVE ##
###########################

visoka_gradnja = Strategy(color=Color.YELLOW,
                          square=Square.UPPER,
                          mood=Mood.AGGRESSIVE)

#################
## PREPARATION ##
#################

visoka_gradnja(
    task_steps=sima_coordinates(
        sima1_coor=[Position(1000, 1000, 1.57, 100), Position(
            1500, 1500, 0, 100), Position(900, 900.5, 0, 200)],
        sima2_coor=[],
        sima3_coor=[],
        sima4_coor=[Position(125, -400, 0, 0), Position(1000, -600, 0, 35), Position(1800, -550, 0, 35)])
)

visoka_gradnja(
    task_steps=init_position(
        Area.YELLOW_HOME.x + 77.5,
        Area.YELLOW_HOME.y - 48,
        3.14,
        'left corner')
)

######################
## PICK-UP STACK 9 ##
######################

visoka_gradnja(ID=1,
               m=Move.Spline([MaterialStack.STACK9.x],  # - 10],
                             [MaterialStack.STACK9.y+350],
                             [3.14],
                             800,
                             'f'))

visoka_gradnja(m=Move.RotateTo(-1.57, 15, 10),
               task_steps=init_front_servos())

visoka_gradnja(task_steps=pickup_front_full_stack(375))

######################################
## SEPARATE STACK 9 IN YELLOW AREA 2 ##
######################################

visoka_gradnja(m=Move.Spline([Area.YELLOW_2.x + 50],
                             [Area.YELLOW_2.y + 120],
                             [-1.57],
                             400,
                             'f'),
               task_steps=two_level())

visoka_gradnja(task_steps=drop_separate_two_level(-275))

#####################
## PICK-UP STACK 5 ##
#####################

visoka_gradnja(m=Move.To(MaterialStack.STACK5.x,
                         MaterialStack.STACK5.y + 400,
                         'f',
                         1500, 1500, 15, 15),
               task_steps=init_front_servos())

visoka_gradnja(m=Move.RotateTo(-1.57, 15, 5))

visoka_gradnja(task_steps=pickup_front_full_stack(375))

#################################
## LEAVE BANNER IN YELLOW AREA 4 ##
#################################

# Already in position
visoka_gradnja(task_steps=leave_banner())

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN YELLOW AREA 2                  ##
##############################################

visoka_gradnja(m=Move.To(MaterialStack.STACK5.x+50,
                         325 - 10,
                         'f', 1000, 500, 10, 5),
               task_steps=init_back_servos())

visoka_gradnja(m=Move.RotateTo(0.0, 5, 5),
               task_steps=two_level())

visoka_gradnja(task_steps=lift_two_on_one(300, -135))
visoka_gradnja(task_steps=push_two_level(push_distance=135))

#####################
## PICK-UP STACK4 ##
#####################

visoka_gradnja(m=Move.To(MaterialStack.STACK4.x + 300,
                         MaterialStack.STACK4.y + 30,
                         'f', 1500, 1500, 15, 15),
               task_steps=init_front_servos())

visoka_gradnja(m=Move.RotateTo(0, 15, 10))

visoka_gradnja(task_steps=pickup_front_full_stack(200))

# Make space for rotation while making two levels
visoka_gradnja(m=Move.Distance(-250, 1000, 500))

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN YELLOW AREA 2 - SECOND TIME    ##
##############################################

visoka_gradnja(m=Move.To(MaterialStack.STACK5.x + 50,
                         325 - 10,
                         'f', 1500, 500, 10, 3),
               task_steps=two_level())

visoka_gradnja(m=Move.RotateTo(0.0, 10, 5))

visoka_gradnja(task_steps=lift_two_on_one(300, -135))
visoka_gradnja(task_steps=push_two_level(push_distance=135))

#####################
## PICK-UP STACK 2 ##
#####################

visoka_gradnja(m=Move.To(MaterialStack.STACK2.x,
                         MaterialStack.STACK2.y - 350,
                         'f', 1000, 1500, 15, 15),
               task_steps=init_front_servos())

visoka_gradnja(m=Move.RotateTo(1.57, 15, 3))

visoka_gradnja(task_steps=pickup_front_full_stack())

###############################
## PICK-UP STACK 3 WITH BACK ##
###############################

visoka_gradnja(m=Move.Spline([MaterialStack.STACK3.x + 225],
                             [MaterialStack.STACK3.y - 15],
                             [3.14],
                             450,
                             'r'))

visoka_gradnja(m=Move.RotateTo(0.0, 15, 5))
visoka_gradnja(task_steps=pickup_back_full_stack())
visoka_gradnja(m=Move.Distance(100, 500, 300))

##########################
## MOVE TO YELLOW AREA 2  ##
##########################

visoka_gradnja(m=Move.To(MaterialStack.STACK5.x+50,
                         325 - 10,
                         'f', 1000, 500, 10, 5),
               task_steps=two_level())

visoka_gradnja(m=Move.RotateTo(0.0, 5, 5))

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN YELLOW AREA 2 - THIRD TIME     ##
##############################################

visoka_gradnja(task_steps=lift_two_on_one(300, -200))

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN BLUE AREA 2 - FOURTH TIME      ##
##############################################


visoka_gradnja(m=Move.RotateTo(1.57, 10, 5),
               s=[Servo.BackLift(BackGripLift.DOWN, 30)],
               task_steps=init_front_servos())

visoka_gradnja(m=Move.Distance(200, 500, 500),
               s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
                  Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN)])

visoka_gradnja(m=Move.RotateTo(-1.57, 15, 10))
visoka_gradnja(task_steps=pickup_front_full_stack(280))
visoka_gradnja(task_steps=two_level())
visoka_gradnja(task_steps=drop_two_level())


##########
## HOME ##
##########
visoka_gradnja(ID=100,
               task_steps=open_all())

visoka_gradnja(m=Move.Spline([Area.YELLOW_HOME.x],
                             [Area.YELLOW_HOME.y-250],
                             [-2.35],
                             800,
                             'r'),
               task_steps=init_all_servos(),
               c=[Condition.InPosition(102)])

# KEEP AT BOTTOM OF STARTEGY - ensures points for home are given when movement is done
visoka_gradnja(ID=102, p=10)
