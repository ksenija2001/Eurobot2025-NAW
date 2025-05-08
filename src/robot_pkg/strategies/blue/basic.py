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
        sima1_coor=[Position(2875, -1605, 180, 0),
                    Position(2000, -1300, 0, 50),
                    Position(1200, -1400, 0, 50)],
        sima2_coor=[Position(2875, -1724, 180, 0),
                    Position(2700, -1724, 0, 50),
                    Position(2100, -1400, 0, 50),
                    Position(1700, -1360, 0, 50)],
        sima3_coor=[Position(2875, -1815, 180, 0),
                    Position(2500, -1815, 0, 50),
                    Position(2100, -1500, 0, 50)],
        sima4_coor=[Position(2875, -1915, 180, 0),
                    Position(1700, -1915, 0, 20),
                    Position(1700, -1415, 0, 4)])
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
                    550,
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

# basic(m=Move.Distance(-350, 1500, 500),
#       task_steps=two_level())
# basic(task_steps=drop_one_level(p=-4))
# basic(m=Move.RotateTo(3.14, 15, 15),
#       task_steps=close_back())
# basic(task_steps=pickup_back_full_stack(-220))

basic(m=Move.Distance(-350, 800, 500),
      task_steps=two_level())

basic(task_steps=separate_two_level(rotation=3.14))

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
# task_steps = drop_back_one_level()
basic(m=Move.RotateTo(1.57, 15, 10))  # 5, 5))

basic(task_steps=pickup_front_full_stack(315, ID=5))

# TODO ALTERNATIVA ID=5 - NEMA 10 - ide se odma na STACK 1, ostavljaa se trospratnica u BLUE area 2 i ide se na 8 posle

################################################
## LEAVE LOWER LEVEL FROM BACK IN BLUE AREA 2 ##
################################################

basic(m=Move.To(Area.BLUE_2.x,
                Area.BLUE_2.y,
                'r', 1000, 500, 10, 5),
      task_steps=two_level())

basic(task_steps=drop_back_one_level(rotation=1.57))


# basic(m=Move.RotateTo(1.57, 5, 5),
#       s=[Servo.BackLift(BackGripLift.DOWN)])
# basic(task_steps=open_back())

# basic(m=Move.Distance(250, 500, 300))

###################################
## BUILD TWO LEVELS ON TOP OF IT ##
###################################

basic(m=Move.RotateTo(-1.57, 5, 3))

basic(task_steps=lift_two_on_one(250))

basic(task_steps=init_front_servos())

#####################
##  PICKUP STACK 1 ##
#####################

basic(m=Move.To(MaterialStack.STACK1.x + 15,
                MaterialStack.STACK1.y - 365,
                'f', 1500, 1500, 15, 15),
      task_steps=close_front())

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

basic(task_steps=pickup_back_full_stack())  # id=5))
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

# basic(s=[Servo.BackLift(BackGripLift.DOWN)])

# basic(m=Move.Distance(390, 500, 500),
#       task_steps=open_back())

basic(task_steps=drop_back_one_level(forward_distance=390, p=-4))

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

basic(task_steps=drop_one_level(p=-4))

basic(m=Move.RotateTo(-1.57, 15, 10),
      task_steps=init_back_servos())

basic(task_steps=pickup_back_full_stack())

basic(task_steps=lift_one_on_two(forward_distance=380+200))

##############################################
## LEAVE ONE LEVEL FROM BACK IN BLUE AREA 2 ##
##############################################

basic(task_steps=drop_back_one_level(rotation=1.57, forward_distance=-100))

# basic(m=Move.RotateTo(1.57, 10, 5))

# basic(m=Move.Distance(-100, 500, 300),
#       task_steps=open_back(),
#       p=Points.LEVEL1)

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
      task_steps=init_front_servos())

basic(m=Move.RotateTo(1.57, 15, 10))  # 5, 5))

basic(task_steps=(pickup_front_full_stack(300)))


###############################
## PICK-UP STACK 5 WITH BACK ##
###############################

basic(m=Move.To(MaterialStack.STACK5.x,
                MaterialStack.STACK5.y + 300,
                'r', 1500, 1000, 15, 10),
      task_steps=init_back_servos())

basic(m=Move.RotateTo(1.57, 15, 10))

basic(task_steps=pickup_back_full_stack())

########################################
## LEAVE STACK 5 IN FRONT OF BLUE AREA 3 ##
########################################

basic(m=Move.Spline([Area.BLUE_3.x + 450],
                    [Area.BLUE_3.y],
                    [3.14],
                    400,
                    'f'),
      task_steps=two_level())

basic(task_steps=drop_back_one_level(forward_distance=390, p=-4))

########################################
## BUILD THREE LEVELS FROM STACK 10 AND STACK 5 IN BLUE AREA 3 ##
########################################

basic(task_steps=drop_two_level())

basic(m=Move.RotateTo(0, 15, 10),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(325))

basic(task_steps=two_level())

basic(task_steps=drop_one_level(p=-4))

basic(m=Move.RotateTo(3.14, 15, 10),
      task_steps=init_back_servos())

basic(task_steps=pickup_back_full_stack())

basic(task_steps=lift_one_on_two(forward_distance=380+200))

basic(task_steps=init_front_servos())

##############################################
## LEAVE ONE LEVEL FROM BACK IN BLUE AREA 3 ##
##############################################

basic(task_steps=drop_back_one_level(rotation=0, forward_distance=-100))

basic(m=Move.Distance(300, 500, 300),
      task_steps=init_front_servos())

basic(m=Move.RotateTo(-1.57, 15, 10))

################################
## PICK-UP STACK 4 WITH FRONT ##
################################

basic(m=Move.Spline([MaterialStack.STACK4.x + 225],
                    [MaterialStack.STACK4.y - 10],
                    [3.14],
                    400,
                    'f'),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(100))

basic(m=Move.Distance(-150, 500, 500))

# basic(m=Move.RotateTo(3.14, 10, 5))

####################################
## LIFT STACK 4 ON ONE LEVEL IN BLUE AREA 3 ##
####################################

basic(m=Move.Spline([Area.BLUE_3.x + 650],
                    [Area.BLUE_3.y],
                    [3.14],
                    300,
                    'r'),
      task_steps=two_level())

basic(task_steps=lift_two_on_one(forward_distance=440))

basic(m=Move.RotateTo(0, 15, 15),
      task_steps=init_front_servos())


#####################
## PICK-UP STACK 1 ##
#####################

# TODO proveriti da li je oko 85s
basic(m=Move.Spline([1500, MaterialStack.STACK2.x + 10],
                    [1250, MaterialStack.STACK2.y - 400],
                    [0, 1.57],
                    800,
                    'f'),
      task_steps=close_front())

basic(task_steps=pickup_front_full_stack(300))

#####################
## LEAVE IN BLUE AREA 2 ##
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

basic(m=Move.RotateTo(0, 10, 5))

#################################
## PICK-UP STACK 6             ##
#################################

basic(m=Move.Spline([MaterialStack.STACK5.x + 25],
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
## NO STACK 10 ALTERNATIVE - ID=5 ##
####################################

basic(ID=5)

################################################
## LEAVE LOWER LEVEL FROM BACK IN BLUE AREA 2 ##
################################################

basic(m=Move.To(Area.BLUE_2.x,
                Area.BLUE_2.y + 20,
                'r', 1000, 500, 10, 5))

basic(m=Move.RotateTo(1.57, 10, 5),
      s=[Servo.BackLift(BackGripLift.DOWN)])
basic(task_steps=open_back())

basic(m=Move.Distance(250, 500, 300))

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
                    [MaterialStack.STACK8.y - 20],
                    [3.14],
                    450,
                    'r'))

basic(task_steps=pickup_back_full_stack())  # id=6))
# TODO nema STACK 8 - ostaviti dvospratnicu od STACK 1 u BLUE AREA 2

basic(m=Move.Distance(250, 1000, 500),
      s=[Servo.BackLift(BackGripLift.UP2)])

##########################
##  MOVE TO BLUE AREA 2 ##
##########################

# TODO brzi spline i slaganje tek kada stigne zbog obima
basic(m=Move.Spline([Area.BLUE_2.x],
                    [Area.BLUE_2.y + 450],
                    [-1.57],
                    500,
                    'f'),
      task_steps=two_level())

#####################################################
## LEAVE STACK 8 FROM BACK IN FRONT OF BLUE AREA 2 ##
## DROP TWO LEVELS IN BLUE AREA 2                  ##
#####################################################

basic(s=[Servo.BackLift(BackGripLift.DOWN)])

basic(task_steps=open_back())

basic(task_steps=lift_two_on_one(forward_distance=420, back_distance=-200))


#######################################
## PICK-UP STACK 8 WHERE IT WAS LEFT ##
## AND BUILD TWO LEVELS              ##
#######################################

basic(m=Move.RotateTo(1.57, 15, 10),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(250))

basic(m=Move.RotateTo(-1.57, 10, 5),
      task_steps=two_level())

basic(m=Move.Distance(300, 500, 500))

basic(task_steps=drop_two_level(back_distance=-350))

# ###############################################
# ## PICK-UP LOWER WITH BACK                   ##
# ## LIFT UPPER ON CONSTRUCTION IN BLUE AREA 2 ##
# ###############################################

# basic(task_steps=drop_one_level(p=-4))

# basic(m=Move.RotateTo(-1.57, 15, 10),
#       task_steps=init_back_servos())

# basic(task_steps=pickup_back_full_stack())

# basic(task_steps=lift_one_on_two(forward_distance=380+200, backout_distance=-350))

##############################################
## PICK-UP STACK 4 ##
##############################################

basic(m=Move.RotateTo(3.14, 15, 15))

basic(m=Move.Distance(800, 1000, 1000))

# Proveriti vreme
basic(m=Move.To(MaterialStack.STACK4.x + 300, 
                  MaterialStack.STACK4.y-15, 
                  'f', 1500, 1500, 15, 10),
      task_steps=(init_front_servos()))

basic(m=Move.RotateTo(3.14, 10, 5))

basic(task_steps=pickup_front_full_stack(forward_distance=200, ID=7))

basic(m=Move.Distance(-150, 500, 500))

basic(m=Move.RotateTo(1.57, 10, 5))

basic(m=Move.Distance(400, 500, 500),
      task_steps=two_level())

basic(m=Move.RotateTo(3.14, 10, 5))

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

basic(m=Move.To(Area.BLUE_HOME.x - 300, 
                  Area.BLUE_HOME.y - 500, 
                  'f', 1100, 1500, 15, 15),
      task_steps=init_all_servos())

# Wait for 99s to enter area
basic(c=[Condition.MatchTime(101, 99)])

basic(ID=101,
      m=Move.To(Area.BLUE_HOME.x  - 300, 
                  Area.BLUE_HOME.y - 300, 
                  'f', 1100, 1500, 15, 15),
      p=10)
