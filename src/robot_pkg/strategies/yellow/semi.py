from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *

#################################
## BLUE CENTER SEMI AGGRESSIVE ##
#################################

semi = Strategy(color=Color.BLUE, square=Square.CENTER, mood=Mood.SEMI)

#################
## PREPARATION ##
#################

semi(
    task_steps=sima_coordinates(
        sima1_coor=[Position(1000, 1000, 1.57, 100), Position(
            1500, 1500, 0, 100), Position(900, 900.5, 0, 200)],
        sima2_coor=[],
        sima3_coor=[],
        sima4_coor=[Position(125, -400, 0, 0), Position(1000, -600, 0, 35), Position(1800, -550, 0, 35)])
)
semi(task_steps=init_position(Area.BLUE_3.x + 48,
                              Area.BLUE_3.y - 77.5,
                              -1.57,
                              'middle',
                              -0.5))

#####################
## PICK-UP STACK 5 ##
#####################

semi(ID=1, m=Move.To(MaterialStack.STACK5.x,
                     MaterialStack.STACK5.y + 350,
                     'f',
                     1500, 1500, 15, 10))

semi(m=Move.RotateTo(-1.57, 15, 10),
     task_steps=init_front_servos())

semi(task_steps=pickup_front_full_stack())

#################################
## LEAVE BANNER IN BLUE AREA 5 ##
#################################

semi(m=Move.Spline([Area.BLUE_5.x + 75],
                   [Area.BLUE_5.y + 175],
                   [1.57],
                   400,
                   'r'))

semi(task_steps=leave_banner(back_distance=-150))

#################################
## DROP STACK 5 IN BLUE AREA 3 ##
#################################

semi(m=Move.Spline([Area.BLUE_3.x],
                   [Area.BLUE_3.y-25],
                   [3.14],
                   350, 'f'),

     task_steps=two_level())

semi(task_steps=drop_two_level())

#####################
## PICK-UP STACK 3 ##
#####################

semi(m=Move.To(Area.BLUE_3.x + 175,
               MaterialStack.STACK3.y,
               'f', 1500, 1000, 15, 10),
     task_steps=init_front_servos())

semi(m=Move.RotateTo(3.14, 10, 5))

semi(task_steps=pickup_front_full_stack())

semi(m=Move.Distance(-200, 500, 300))

#########################################
## DROP HALF OF STACK 3 in BLUE AREA 3 ##
## AND LIFT OTHER HALF ON TWO LEVEL    ##
#########################################

semi(m=Move.To(Area.BLUE_3.x + 175,
               Area.BLUE_3.y-25,
               'f',
               1500, 1000, 10, 5),
     task_steps=two_level())

semi(task_steps=drop_one_level(-100, p=-4))
semi(m=Move.RotateTo(3.14, 15, 5))  # MOZE BRZE AKO NIJE PREBLIZU

semi(task_steps=lift_one_on_two())

#####################
## PICK-UP STACK 8 ##
#####################

semi(m=Move.RotateTo(0.707, 15, 10),
     task_steps=init_front_servos())

semi(m=Move.Spline([1200, MaterialStack.STACK8.x-290],
                   [1250, MaterialStack.STACK8.y],
                   [0.1, 0],
                   900,
                   'f'))

semi(task_steps=pickup_front_full_stack(200))

###############################
## PICK-UP STACK 1 WITH BACK ##
###############################

semi(m=Move.Spline([MaterialStack.STACK1.x],
                   [MaterialStack.STACK1.y-325],
                   [-1.57],
                   450,
                   'r'),
     task_steps=init_back_servos())

semi(task_steps=pickup_back_full_stack())

# semi(m=Move.Distance(300, 1500, 1500))

#################################
## DROP STACK 8 IN BLUE AREA 2 ##
#################################

# Moving through STACK 10
semi(m=Move.Spline([Area.BLUE_2.x],
                   [Area.BLUE_2.y + 200],
                   [-1.57],
                   350,
                   'f'),
     s=[Servo.BackLift(BackGripLift.UP)])

semi(m=Move.Distance(-100, 500, 500),
     task_steps=two_level())

semi(task_steps=drop_two_level(-200))

semi(m=Move.RotateTo(0, 10, 5))

#################################
## PICK-UP STACK 6             ##
#################################

semi(m=Move.Spline([MaterialStack.STACK6.x + 25],
                   [MaterialStack.STACK6.y + 250],
                   [-1.57],
                   400,
                   'f'),
     task_steps=init_front_servos())

semi(task_steps=pickup_front_full_stack(300))
semi(task_steps=two_level())
semi(task_steps=drop_one_level(-250))

#################################
## LEAVE STACK 1 FORM BACK     ##
#################################

semi(m=Move.RotateTo(1.57, 3, 3),
     s=[Servo.BackLift(BackGripLift.DOWN)])

semi(m=Move.Distance(100, 500, 300),
     task_steps=open_back())

##########################################
## RETURN FOR LEFT STACK IN BLUE AREA 2 ##
##########################################

semi(m=Move.Spline([Area.BLUE_2.x - 25],
                   [Area.BLUE_2.y+450],
                   [-1.57],
                   400,
                   'f'))

semi(task_steps=lift_one_on_two(150))

################################################
## RETURN FOR LEFT STACK 1 IN FRONT OF AREA 4 ##
################################################

semi(m=Move.RotateTo(0, 15, 15))

semi(m=Move.Spline([MaterialStack.STACK6.x + 25],
                   [MaterialStack.STACK6.y + 350],
                   [-1.57],
                   400,
                   'f'),
     task_steps=init_front_servos())

semi(task_steps=pickup_front_full_stack())
semi(task_steps=two_level())
semi(task_steps=lift_two_on_one())

##########
## HOME ##
##########

semi(ID=100,
     task_steps=open_all())

semi(m=Move.Spline([Area.BLUE_HOME.x],
                   [Area.BLUE_HOME.y-250],
                   [-1.57],
                   800,
                   'r'),
     task_steps=init_all_servos(),
     c=[Condition.InPosition(102)])


# KEEP AT BOTTOM OF STARTEGY - ensures points for home are given when movement is done
semi(ID=102, p=10)
