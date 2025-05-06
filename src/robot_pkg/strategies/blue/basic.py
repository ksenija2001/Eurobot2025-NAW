from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *

########################
## BLUE LOWER PASSIVE ##
########################

basic = Strategy(color=Color.BLUE, square=Square.LOWER, mood=Mood.PASSIVE)

#################
## PREPARATION ##
#################

basic(
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
      task_steps=init_front_servos(),
      c=[Condition.Detection(2, attempts=0),])
# TODO DETEKCIJA ID=2 - ALTERNATIVA NA STACK 10 - ide se na 5, 4, 3
# TODO DETEKCIJA ID=3 - ALTERNATIVA DETEKCIJE NA 10 - ide se na 9, pa 5, 4, 3

basic(m=Move.RotateTo(-1.57, 10, 10))

# NEMOGUCE DA NEMA STACK 6 A DA SE NIJE DESILA DETEKCIJA
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
basic(m=Move.RotateTo(-1.57, 10, 5))  # 5, 5))

basic(task_steps=lift_one_on_two(forward_distance=250))

######################
## PICK-UP STACK 10 ##
######################

basic(m=Move.To(MaterialStack.STACK10.x + 10,
                MaterialStack.STACK10.y - 350,
                'f', 1500, 1000, 15, 10),  # 5, 3),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(1.57, 10, 5))  # 5, 5))

basic(task_steps=(pickup_front_full_stack(315)))#, id=4)))

# TODO ALTERNATIVA ID=4 - NEMA 10 - ide se odma na STACK 1, ostavljaa se trospratnica u BLUE area 2 i ide se na 8 posle

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

basic(task_steps=pickup_back_full_stack())#id=5))
# TODO nema STACK 8 - ostaviti dvospratnicu od STACK 1 u BLUE AREA 2

basic(m=Move.Distance(250, 1000, 500),
      s=[Servo.BackLift(BackGripLift.UP2)])

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

basic(s=[Servo.BackLift(BackGripLift.DOWN)])

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

basic(m=Move.Distance(100, 500, 300),
      c=[Condition.InPosition(100),])


#############################################
## DETECTION ON STACK 6 ALTERNATIVE - ID 2 ##
#############################################
# ALTERNATIVA NA STACK 10 - ide se na 5, 4, 3

basic(ID=2)

######################
## PICK-UP STACK 10 ##
######################

basic(m=Move.To(MaterialStack.STACK10.x + 10,
                MaterialStack.STACK10.y - 350,
                'f', 1500, 1000, 15, 10),  # 5, 3),
      task_steps=init_front_servos(),
      c=[Condition.Detection(3, attempts=2),])

basic(m=Move.RotateTo(1.57, 15, 10))  # 5, 5))

basic(task_steps=(pickup_front_full_stack(300)))

basic(c=[Condition.Timeout(4, 0.1),])

##############################################
## DETECTION ON STACK 10 ALTERNATIVE - ID 3 ##
##############################################
# ALTERNATIVA DETEKCIJE NA 10 - ide se na 9, pa 5, 4, 3

basic(ID=3)

#####################
## PICK-UP STACK 9 ##
#####################

basic(m=Move.To(MaterialStack.STACK9.x + 10,
                MaterialStack.STACK9.y - 350,
                'f', 1500, 1000, 15, 10),  # 5, 3),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(1.57, 15, 10))  # 5, 5))

basic(task_steps=(pickup_front_full_stack(300)))


#############################################
## MERGE ID 2 AND ID 3 ALTERNATIVES - ID 4 ##
#############################################
# ALTERNATIVA DETEKCIJE NA 10 - ide se na 9, pa 5, 4, 3

basic(ID=4)

###############################
## PICK-UP STACK 5 WITH BACK ##
###############################

basic(m=Move.To(MaterialStack.STACK5.x,
                MaterialStack.STACK5.y + 300,
                'r', 1500, 1000, 15, 10),
      task_steps=init_back_servos())

basic(m=Move.RotateTo(1.57, 15, 10))

basic(task_steps=pickup_back_full_stack())

###################################
## SEPARATE STACK IN BLUE AREA 3 ##
###################################

basic(m=Move.Spline([Area.BLUE_3.x + 120],
                    [Area.BLUE_3.y],
                    [3.14],
                    400,
                    'f'),
      s=[Servo.BackLift(BackGripLift.UP2)],
      task_steps=two_level())

basic(task_steps=drop_separate_two_level(-275, backout_distance=-300))

basic(m=Move.RotateTo(-1.57, 15, 10))

################################
## PICK-UP STACK 4 WITH FRONT ##
################################

basic(m=Move.Spline([MaterialStack.STACK4.x + 175],
                    [MaterialStack.STACK4.y],
                    [3.14],
                    400,
                    'f'),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack())

basic(m=Move.Distance(-100, 500, 500))

####################################
## LEAVE BACK AND LIFT TWO ON ONE ##
####################################

basic(m=Move.RotateTo(1.57, 5, 5),
      task_steps=two_level(),
      s=[Servo.BackLift(BackGripLift.DOWN)])

basic(task_steps=open_back())

basic(task_steps=lift_two_on_one(forward_distance=250, back_distance=-135))
basic(task_steps=push_two_level(push_distance=135, backout_distance=-250))

##############################################
## PICK-UP LEFT STACK 5 AND LIFT TWO ON ONE ##
##############################################

basic(m=Move.RotateTo(-1.57, 15, 10),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(150))

basic(m=Move.RotateTo(1.57, 5, 5),
      task_steps=two_level())

basic(task_steps=lift_two_on_one(300, -300))


#####################
## PICK-UP STACK 3 ##
#####################

basic(m=Move.RotateTo(0, 15, 15))
basic(m=Move.Spline([MaterialStack.STACK3.x + 175],
                    [MaterialStack.STACK3.y],
                    [3.14],
                    400, 'f'),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack())

basic(m=Move.Distance(-100, 500, 500))

basic(m=Move.RotateTo(-1.57, 10, 5),
      task_steps=two_level())

basic(task_steps=lift_two_on_one(250, -300))

basic(m=Move.RotateTo(0, 15, 15),
      c=[Condition.InPosition(100),])


##########
## HOME ##
##########

basic(ID=100,
      m=Move.Distance(150, 1000, 1000),
      task_steps=open_all())

basic(m=Move.To(Area.BLUE_HOME.x, Area.BLUE_HOME.y - 450, 'f', 1100, 1500, 15, 15),
      task_steps=init_all_servos())

# Wait for 99s to enter area
basic(c=[Condition.MatchTime(101, 99)])

basic(ID=101,
      m=Move.To(Area.BLUE_HOME.x, Area.BLUE_HOME.y -
                250, 'f', 1100, 1500, 15, 15),
      p=10)
