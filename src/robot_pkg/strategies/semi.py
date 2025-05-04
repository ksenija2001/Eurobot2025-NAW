from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *

##########################
## BLUE SEMI AGGRESSIVE ##
##########################

semi = Strategy(color = Color.BLUE, square = Square.CENTER, mood = Mood.SEMI)

# semi(sima_id=4, 
#   sima=[Position(125, -400, 0, 0), 
#         Position(1000, -600, 0, 35), 
#         Position(1800, -550, 0, 35)])
# # semi(sima_id=1, 
# #   sima=[Position(1000, 1000, 1.57, 100), Position(1500, 1500, 0, 100), Position(900, 900.5, 0, 200)])

semi(task_steps=init_position(Area.BLUE_3.x + 48, 
                            Area.BLUE_3.y - 77.5, 
                            -1.57, 
                            'middle'))

#####################
## PICK-UP STACK 5 ##
#####################

semi(ID=1, m=Move.Spline([MaterialStack.STACK5.x],
                    [MaterialStack.STACK5.y + 250],
                    [-1.57],
                    500,
                    'f'),
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

# semi(m=Move.To(Area.BLUE_5.x + 125, 
#                 Area.BLUE_3.y + 80,
#                 'f',
#                 1500, 1000, 15, 10),
#     task_steps=two_level())

semi(m=Move.Spline([Area.BLUE_3.x],
                    [Area.BLUE_3.y],
                    [3.14],
                    350, 'f'),
                    
    task_steps=two_level())

# semi(m=Move.RotateTo(3.14, 15, 10))
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
                Area.BLUE_3.y,
                'f',
                1500, 1000, 15, 10),
    task_steps=two_level())   

semi(task_steps=drop_one_level(-100))    
semi(m=Move.RotateTo(3.14, 15, 5)) # MOZE BRZE AKO NIJE PREBLIZU

semi(task_steps=lift_one_on_two())

#####################
## PICK-UP STACK 8 ##
#####################

semi(m=Move.RotateTo(0.707, 15, 10),
    task_steps=init_front_servos())

semi(m=Move.Spline([1000, MaterialStack.STACK8.x-290],
                    [1250, MaterialStack.STACK8.y],
                    [0.1, 0],
                    500, 
                    'f'))

semi(task_steps=pickup_front_full_stack(200))

###############################
## PICK-UP STACK 1 WITH BACK ##
###############################

semi(m=Move.Spline([MaterialStack.STACK1.x],
                    [MaterialStack.STACK1.y-350],
                    [-1.57],
                    400,  
                    'r'),
    task_steps=init_back_servos())

semi(task_steps=pickup_back_full_stack())

# semi(m=Move.Distance(300, 1500, 1500))

##########################################
## PUSH WITH FRONT STACK 10 IF IT EXIST ##
##########################################

semi(m=Move.Spline([MaterialStack.STACK10.x],
                    [MaterialStack.STACK10.y],
                    [-1.57],
                    350,
                    'f'),
    task_steps=front_lift_stack()) 

#################################
## DROP STACK 8 IN BLUE AREA 2 ##
## FIRST ALTERNATIVE           ##
#################################

semi(m=Move.Spline([Area.BLUE_2.x],
                    [Area.BLUE_2.y],
                    [-1.57],
                    400,
                    'f'),
    c=[Condition.FrontSensors(2)])     ## SKIP TO ID=2 IF THERE ARE NO CANS)

semi(m=Move.Distance(-150, 300, 300),
    task_steps=two_level())

semi(task_steps=lift_two_on_one())  

semi(m=Move.RotateTo(1.57, 10, 10))

semi(s=[Servo.BackLift(BackGripLift.DOWN)])

#################################
## DROP STACK 1 IN BLUE AREA 2 ##
## PICK-UP STACK 6             ##
#################################

semi(m=Move.Spline([MaterialStack.STACK6.x],
                   [MaterialStack.STACK6.y + 250],
                   [-1.57],
                   400,
                   'f'),
    task_steps=init_front_servos(),
    s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN), 
        Servo.BackCenterGrip(BackCenterLeft.OPEN, BackSideRight.OPEN)])

semi(task_steps=pickup_front_full_stack())
semi(task_steps=two_level())
semi(task_steps=drop_two_level())

#####################
## PICK-UP STACK 7 ##
#####################

semi(m=Move.To(MaterialStack.STACK7.x - 290, 
             MaterialStack.STACK7.y, 
             'f', 1500, 1500, 15, 15),
    task_steps=init_front_servos())

semi(pickup_front_full_stack())
semi(m=Move.Distance(-200, 500, 300),
        task_steps=two_level())

semi(task_steps=drop_one_level())

semi(m=Move.RotateTo(3.14, 15, 10),
    task_steps=init_back_servos())

semi(task_steps=pickup_back_full_stack())

#########################################
## LIFT HALF OF STACK 7 IN BLUE AREA 4 ##
#########################################
semi(m=Move.Spline([MaterialStack.STACK6.x],
                   [MaterialStack.STACK6.y + 150],
                   [-1.57],
                   400,
                   'f'))
# semi(m=Move.RotateTo(-1.57, 10, 5))

semi(task_steps=lift_one_on_two())

semi(m=Move.RotateTo(1.57, 5, 5))

##########################################
## RETURN FOR LEFT STACK IN BLUE AREA 2 ##
##########################################

semi(m=Move.Spline([Area.BLUE_2.x],
                    [Area.BLUE_2.y+350],
                    [-1.57],
                    400,
                    'f'),
    task_steps=init_front_servos())

semi(task_steps=pickup_front_full_stack())

semi(m=Move.RotateTo(1.57, 5, 5),
    task_steps=two_level())

#############################################
## LEAVE HALF OF STACK 7 FROM BACK GRIPPER ##
#############################################

semi(m=Move.Distance(-100, 300, 300),
    s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN), 
        Servo.BackCenterGrip(BackCenterLeft.OPEN, BackSideRight.OPEN)])

semi(m=Move.Distance(150, 500, 300))

semi(m=Move.RotateTo(-1.57, 10, 5))

semi(task_steps=lift_two_on_one())

##########
## HOME ##
##########

semi(ID=100,
    m=Move.Spline([Area.BLUE_HOME.x], 
                [Area.BLUE_HOME.y-250], 
                [2.35],
                800,
                'f'))















#################################
## DROP STACK 8 IN BLUE AREA 2 ##
## SECOND ALTERNATIVE          ##
#################################

# semi(ID=2,                            ## WILL BE SKIPPED IF CANS WERE FOUND
#     m=Move.To(Area.BLUE_2.x, Area.BLUE_2.y+50, 'r', 1000, 500, 10, 5),
#     task_steps=two_level())

# semi(m=Move.RotateTo(1.57, 10, 5),
#     s=[Servo.BackLift(BackGripLift.DOWN)])

# semi(m=Move.Distance(150, 500, 300),
#     task_steps=open_back())

# semi(m=Move.RotateTo(-1.57, 15, 10))
# semi(task_steps=lift_two_on_one())






