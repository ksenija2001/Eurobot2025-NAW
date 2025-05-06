from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *

#################################
## YELLOW CENTER SEMI AGGRESSIVE ##
#################################

semi = Strategy(color=Color.YELLOW, square=Square.CENTER, mood=Mood.SEMI)

#################
## PREPARATION ##
#################

semi(
    task_steps=sima_coordinates(
        sima1_coor=[Position(1000, 1000, 1.57, 100),
                    Position(1500, 1500, 0, 100),
                    Position(900, 900.5, 0, 200)],
        sima2_coor=[Position(125, -276, 0, 0), 
                    Position(300, -276, 0, 50), 
                    Position(900, -600, 0, 50), 
                    Position(1200, -580, 0, 50)],
        sima3_coor=[],
        sima4_coor=[Position(125, -400, 0, 0),
                    Position(1000, -600, 0, 35),
                    Position(1800, -550, 0, 35)])
)

# TODO back more and close grippers beause of vertical projection
semi(task_steps=init_position(Area.YELLOW_3.x - 48,
                              Area.YELLOW_3.y + 77.5, # NE ZNAM TREBA LI OVDE +
                              1.57,
                              'middle',
                              -3.14+0.5))

#####################
## PICK-UP STACK 6 ##
#####################

semi(ID=1, m=Move.To(MaterialStack.STACK6.x,
                     MaterialStack.STACK6.y + 350,
                     'f',
                     1500, 1500, 15, 10))

semi(m=Move.RotateTo(-1.57, 15, 10),
     task_steps=init_front_servos())

semi(task_steps=pickup_front_full_stack())

#################################
## LEAVE BANNER IN YELLOW AREA 5 ##
#################################

semi(m=Move.Spline([Area.YELLOW_5.x - 75],
                   [Area.YELLOW_5.y + 175],
                   [1.57],
                   400,
                   'r'))

semi(task_steps=leave_banner(back_distance=-150))

#################################
## DROP STACK 6 IN YELLOW AREA 3 ##
#################################

semi(m=Move.Spline([Area.YELLOW_3.x],
                   [Area.YELLOW_3.y-25],
                   [0],
                   350, 'f'),

     task_steps=two_level())

semi(task_steps=drop_two_level())

#####################
## PICK-UP STACK 8 ##
#####################

semi(m=Move.To(Area.YELLOW_3.x - 175,
               MaterialStack.STACK8.y,
               'f', 1500, 1000, 15, 10),
     task_steps=init_front_servos())

semi(m=Move.RotateTo(0.0, 10, 5))

semi(task_steps=pickup_front_full_stack())

semi(m=Move.Distance(-200, 500, 300))

#########################################
## DROP HALF OF STACK 8 in YELLOW AREA 3 ##
## AND LIFT OTHER HALF ON TWO LEVEL    ##
#########################################

semi(m=Move.To(Area.YELLOW_3.x - 175,
               Area.YELLOW_3.y-25,
               'f',
               1500, 1000, 10, 5),
     task_steps=two_level())

semi(task_steps=drop_one_level(-100, p=-4))
semi(m=Move.RotateTo(0, 15, 5))  # MOZE BRZE AKO NIJE PREBLIZU

semi(task_steps=lift_one_on_two())

#####################
## PICK-UP STACK 3 ##
#####################

semi(m=Move.RotateTo(2.35, 15, 10),
     task_steps=init_front_servos())

semi(m=Move.Spline([1200, MaterialStack.STACK3.x+290],
                   [1250, MaterialStack.STACK3.y],
                   [3.15, 3.14],
                   900,
                   'f'))

semi(task_steps=pickup_front_full_stack(200))

###############################
## PICK-UP STACK 2 WITH BACK ##
###############################

semi(m=Move.Spline([MaterialStack.STACK2.x],
                   [MaterialStack.STACK2.y-325],
                   [-1.57],
                   450,
                   'r'),
     task_steps=init_back_servos())

semi(task_steps=pickup_back_full_stack())

# semi(m=Move.Distance(300, 1500, 1500))

#################################
## DROP STACK 3 IN YELLOW AREA 2 ##
#################################

# Moving through STACK 9
semi(m=Move.Spline([Area.YELLOW_2.x],
                   [Area.YELLOW_2.y + 200],
                   [-1.57],
                   350,
                   'f'),
     s=[Servo.BackLift(BackGripLift.UP)])

semi(m=Move.Distance(-100, 500, 500),
     task_steps=two_level())

semi(task_steps=drop_two_level(-200))

semi(m=Move.RotateTo(3.14, 10, 5))

#################################
## PICK-UP STACK 5             ##
#################################

semi(m=Move.Spline([MaterialStack.STACK5.x - 25],
                   [MaterialStack.STACK5.y + 250],
                   [-1.57],
                   400,
                   'f'),
     task_steps=init_front_servos())

semi(task_steps=pickup_front_full_stack(300))
semi(task_steps=two_level())
semi(task_steps=drop_one_level(-250))

#################################
## LEAVE STACK 2 FORM BACK     ##
#################################

semi(m=Move.RotateTo(1.57, 3, 3),
     s=[Servo.BackLift(BackGripLift.DOWN)])

semi(m=Move.Distance(100, 500, 300),
     task_steps=open_back())

##########################################
## RETURN FOR LEFT STACK IN YELLOW AREA 2 ##
##########################################

semi(m=Move.Spline([Area.YELLOW_2.x + 25],
                   [Area.YELLOW_2.y + 450],
                   [-1.57],
                   400,
                   'f'))

semi(task_steps=lift_one_on_two(150))

################################################
## RETURN FOR LEFT STACK 2 IN FRONT OF AREA 4 ##
################################################

semi(m=Move.RotateTo(3.14, 15, 15))

semi(m=Move.Spline([MaterialStack.STACK5.x - 25],
                   [MaterialStack.STACK5.y + 150],
                   [-1.57],
                   400,
                   'f'),
     task_steps=init_front_servos())

semi(task_steps=pickup_front_full_stack())
semi(task_steps=two_level())
semi(task_steps=lift_two_on_one())

semi(c=[Condition.Timeout(100, 0.1),])


##########
## HOME ##
##########
semi(ID=100,
      m=Move.Distance(150, 1000, 1000),
      task_steps=open_all())

semi(m=Move.To(Area.YELLOW_HOME.x, Area.YELLOW_HOME.y - 450, 'f', 1100, 1500, 15, 15),
      task_steps=init_all_servos())

# Wait for 99s to enter area
semi(c=[Condition.MatchTime(101, 99)])


semi(ID=101,
      m=Move.To(Area.YELLOW_HOME.x, Area.YELLOW_HOME.y -
                250, 'f', 1100, 1500, 15, 15),
      p=10)
