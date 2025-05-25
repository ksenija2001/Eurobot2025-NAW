from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move
from robot_pkg.opponent import Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *
from robot_pkg.strategies.pickup_tasks import *

########################
## BLUE LOWER PASSIVE ##
########################

basic = Strategy(color=Color.BLUE, square=Square.LOWER, mood=Mood.PASSIVE)

#################
## PREPARATION ##
#################

basic(
    task_steps=sima_coordinates(
        sima1_coor=[Position(2875, 1845, 3.14, 0),
                    Position(2400, 1650, 0, 50),
                    Position(2000-100, 1500-50, 0, 50)],
        sima2_coor=[Position(2875, 1725, 3.14, 0),
                    Position(1865, 1350, 0, 50),
                    Position(1525, 1400, 0, 50)],
        sima3_coor=[Position(2875, 1605, 3.14, 0),
                    Position(1600, 1400, 0, 50),
                    Position(1150, 1450, 0, 50)],
        sima4_coor=[Position(2875,  1905, 3.14, 0),
                    Position(1825, 1855, 0, 50),
                    Position(1700, 1300, 0, 30)])
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

basic(m=Move.Spline([MaterialStack.STACK6.x + 20],
                    [MaterialStack.STACK6.y + 250],
                    [-1.57],
                    550,
                    'f'),
      task_steps=init_front_servos())#,
      #c=[Condition.Detection(2, attempts=0),])

# basic(m=Move.RotateTo(-1.57, 10, 10))

basic(task_steps=pickup_front_full_stack(270))

#####################################
## BUILD TWO LEVELS IN BLUE AREA 4 ##
#####################################

basic(task_steps=two_level())
basic(task_steps=drop_two_level())

#####################
## PICK-UP STACK 7 ##
#####################

basic(task_steps=move_to_front_STACK7())
basic(task_steps=pickup_front_full_stack(200))

####################################
## SEPARATE STACK 7               ##
## PICK-UP LOWER LEVEL WITH BACK  ##
####################################

basic(m=Move.Distance(-50, 1000, 500),
      task_steps=drop_one_in_two_level(rotation=3.14, backout_distance=-175, p=-4))
# basic(m=Move.Distance(-350, 800, 500),
#       task_steps=two_level())

# basic(task_steps=separate_two_level(rotation=3.14))

basic(task_steps=pickup_back_full_stack(-275, forward_distance=50))

#############################################
## MOVE TO BLUE AREA 4 AND LIFT ONE ON TWO ##
#############################################

basic(task_steps=move_to_front_STACK6(init=False))
basic(task_steps=lift_one_on_two(forward_distance=250))

######################
## PICK-UP STACK 10 ##
######################

# ALTERNATIVE 10 - STACK1 -> 3L BLUE AREA 2, STACK 8
basic(c=[Condition.CheckStack('STACK10', 10)])

basic(task_steps=move_to_front_STACK10())
basic(task_steps=pickup_front_full_stack(340, ID=10))

################################################
## LEAVE LOWER LEVEL FROM BACK IN BLUE AREA 2 ##
################################################

basic(m=Move.To(Area.BLUE_2.x,
                Area.BLUE_2.y,
                'r', 1000, 500, 10, 5),
      task_steps=two_level())

basic(task_steps=drop_back_one_level(rotation=1.57))

##################################
# BUILD TWO LEVELS ON TOP OF IT ##
##################################

basic(m=Move.RotateTo(-1.57, 10, 5))

basic(task_steps=lift_two_on_one(250))

#####################
##  PICKUP STACK 1 ##
#####################

basic(task_steps=move_to_front_STACK1())
basic(task_steps=pickup_front_full_stack())

################################
##  PICKUP STACK 8  WITH BACK ##
################################

# ALTERNATIVE 8 - Check STACK9 -> STACK3 -> STACK5
basic(c=[Condition.CheckStack('STACK8', 81)])

basic(m=Move.Spline([MaterialStack.STACK8.x - 230],
                    [MaterialStack.STACK8.y - 35],
                    [3.14],
                    450, 'r'),
      c=[Condition.Detection(81, 1)])

basic(task_steps=pickup_back_full_stack(forward_distance=50, ID=81))

##########################
##  MOVE TO BLUE AREA 2 ##
##########################

# TODO brzi spline i slaganje tek kada stigne zbog obima
basic(m=Move.Spline([Area.BLUE_2.x],
                    [Area.BLUE_2.y + 550],
                    [-1.57],
                    450,
                    'f'),
      task_steps=two_level())

#####################################################
## LEAVE STACK 8 FROM BACK IN FRONT OF BLUE AREA 2 ##
## DROP TWO LEVELS IN BLUE AREA 2                  ##
#####################################################

basic(task_steps=drop_back_one_level(-1.57, forward_distance=390, p=-4))

basic(task_steps=drop_two_level())

#######################################
## PICK-UP STACK 8 WHERE IT WAS LEFT ##
## AND BUILD TWO LEVELS              ##
#######################################

basic(m=Move.RotateTo(1.57, 15, 10),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(300))

basic(task_steps=drop_one_in_two_level(rotation=-1.57, p=-4))
# basic(task_steps=two_level())

###############################################
## PICK-UP LOWER WITH BACK                   ##
## LIFT UPPER ON CONSTRUCTION IN BLUE AREA 2 ##
###############################################

# basic(task_steps=drop_one_level(p=-4))

# basic(m=Move.RotateTo(-1.57, 15, 10),
#       task_steps=init_back_servos())

basic(task_steps=pickup_back_full_stack())

basic(task_steps=lift_one_on_two(forward_distance=380, backout_distance=-200))

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
## DETECTION ON STACK 6 ALTERNATIVE - ID 2 ##
#############################################
# ALTERNATIVA NA STACK 10 - ide se na 5, 4, 3

# basic(ID=2)

# ######################
# ## PICK-UP STACK 10 ##
# ######################

# basic(m=Move.To(MaterialStack.STACK9.x + 10,
#                 MaterialStack.STACK9.y - 350,
#                 'f', 1500, 1000, 15, 10),  # 5, 3),
#       task_steps=init_front_servos())

# basic(m=Move.RotateTo(1.57, 15, 10))  # 5, 5))

# basic(task_steps=(pickup_front_full_stack(300)))


# ###############################
# ## PICK-UP STACK 5 WITH BACK ##
# ###############################

# basic(m=Move.To(MaterialStack.STACK5.x,
#                 MaterialStack.STACK5.y + 250,  # 300,
#                 'r', 1000, 700, 10, 10),
#       task_steps=init_back_servos())

# basic(m=Move.RotateTo(1.57, 15, 10))

# basic(task_steps=pickup_back_full_stack())

# ########################################
# ## LEAVE STACK 5 IN FRONT OF BLUE AREA 3 ##
# ########################################

# basic(m=Move.To(Area.BLUE_3.x + 450,
#                 Area.BLUE_3.y,
#                 'f', 500, 500, 5, 5),
#       task_steps=two_level())
# basic(m=Move.RotateTo(3.14, 5, 5))

# basic(task_steps=drop_back_one_level(rotation=3.14, forward_distance=390, p=-4))

# ########################################
# ## BUILD THREE LEVELS FROM STACK 10 AND STACK 5 IN BLUE AREA 3 ##
# ########################################

# basic(task_steps=drop_two_level())

# basic(m=Move.RotateTo(0, 15, 10),
#       task_steps=init_front_servos())

# basic(task_steps=pickup_front_full_stack(325))

# basic(task_steps=two_level())

# basic(task_steps=drop_one_level(p=-4))

# basic(m=Move.RotateTo(3.14, 15, 10),
#       task_steps=init_back_servos())

# basic(task_steps=pickup_back_full_stack())

# basic(task_steps=lift_one_on_two(forward_distance=380+200))

# basic(task_steps=init_front_servos())

# ##############################################
# ## LEAVE ONE LEVEL FROM BACK IN BLUE AREA 3 ##
# ##############################################

# basic(task_steps=drop_back_one_level(rotation=0, forward_distance=-100))

# basic(m=Move.Distance(300, 500, 300),
#       task_steps=init_front_servos())

# basic(m=Move.RotateTo(-1.57, 15, 10))

# ################################
# ## PICK-UP STACK 4 WITH FRONT ##
# ################################

# basic(m=Move.Spline([MaterialStack.STACK4.x + 225],
#                     [MaterialStack.STACK4.y - 10],
#                     [3.14],
#                     400,
#                     'f'),
#       task_steps=init_front_servos())

# basic(task_steps=pickup_front_full_stack(150))

# basic(m=Move.Distance(-150, 500, 500))

# # basic(m=Move.RotateTo(3.14, 10, 5))

# ####################################
# ## LIFT STACK 4 ON ONE LEVEL IN BLUE AREA 3 ##
# ####################################

# basic(m=Move.Spline([Area.BLUE_3.x + 650],
#                     [Area.BLUE_3.y],
#                     [3.14],
#                     300,
#                     'r'),
#       task_steps=two_level())

# basic(task_steps=lift_two_on_one(forward_distance=440))

# basic(m=Move.RotateTo(0, 15, 15),
#       task_steps=init_front_servos())


# #####################
# ## PICK-UP STACK 1 ##
# #####################

# # TODO proveriti da li je oko 85s
# basic(m=Move.Spline([1500, MaterialStack.STACK1.x + 10],
#                     [1250, MaterialStack.STACK1.y - 400],
#                     [0, 1.57],
#                     550,
#                     'f'),
#       task_steps=close_front())

# basic(task_steps=pickup_front_full_stack(300))

# #####################
# ## LEAVE IN BLUE AREA 2 ##
# #####################

# basic(m=Move.Spline([Area.BLUE_2.x],
#                     [Area.BLUE_2.y + 50],
#                     [1.57],
#                     400,
#                     'r'),
#       s=[Servo.BackSwing(BackSwing.PICK)],
#       task_steps=two_level())

# # basic(task_steps=drop_back_one_level(forward_distance=390, p=-4))
# basic(c=[Condition.BackSensors(10)])
# basic(p=4)

# # No point if STACK 10 wasn't there
# basic(ID=15)

# basic(m=Move.Distance(300, 500, 300))
# basic(m=Move.RotateTo(-1.57, 10, 5))
# basic(task_steps=drop_two_level(-200))

# basic(m=Move.RotateTo(0, 10, 5))

# #################################
# ## PICK-UP STACK 6             ##
# #################################

# basic(m=Move.Spline([MaterialStack.STACK6.x + 25],
#                     [MaterialStack.STACK6.y + 250],
#                     [-1.57],
#                     400,
#                     'f'),
#       task_steps=init_front_servos())

# basic(task_steps=pickup_front_full_stack(250, ID=11))
# basic(task_steps=two_level())
# basic(task_steps=drop_two_level(-250))

# basic(ID=11)

# basic(m=Move.Distance(-100, 500, 300),
#       c=[Condition.InPosition(100),])


####################################
## NO STACK 10 ALTERNATIVE - ID=10 ##
####################################

basic(ID=10)

###############################
## LEAVE BACK IN BLUE AREA 2 ##
###############################

basic(m=Move.To(MaterialStack.STACK10.x + 10,
                MaterialStack.STACK10.y - 350,
                'f', 1500, 1000, 15, 10),
      task_steps=init_front_servos())

basic(m=Move.To(Area.BLUE_2.x,
                Area.BLUE_2.y,
                'r', 1000, 1000, 10, 10))

basic(task_steps=drop_back_one_level(rotation=1.57))

#####################
##  PICKUP STACK 1 ##
#####################

basic(task_steps=move_to_front_STACK1())
basic(task_steps=pickup_front_full_stack())

################################
##  PICKUP STACK 8  WITH BACK ##
################################

# ALTERNATIVE 8 - Check STACK9 -> STACK3 -> STACK5
basic(c=[Condition.CheckStack('STACK8', 82)])

basic(m=Move.Spline([MaterialStack.STACK8.x - 230],
                    [MaterialStack.STACK8.y - 35],
                    [3.14],
                    450, 'r'))

basic(task_steps=pickup_back_full_stack(ID=82))

##########################
##  MOVE TO BLUE AREA 2 ##
##########################

# TODO brzi spline i slaganje tek kada stigne zbog obima
basic(m=Move.Spline([Area.BLUE_2.x],
                    [Area.BLUE_2.y + 450],
                    [-1.57],
                    450,
                    'f'),
      task_steps=two_level())

basic(task_steps=drop_back_one_level(rotation=-1.57, p=-4))

##################################
# BUILD TWO LEVELS ON TOP OF IT ##
##################################

# basic(m=Move.RotateTo(-1.57, 5, 3))

basic(task_steps=lift_two_on_one(250))

basic(m=Move.RotateTo(1.57, 15, 10),
      task_steps=init_front_servos())

basic(task_steps=pickup_front_full_stack(300))

basic(m=Move.RotateTo(-1.57, 10, 10))

basic(m=Move.Distance(320, 500, 500),
      task_steps=two_level())

basic(task_steps=drop_two_level(back_distance=-350))


#################################
## CHECK WHICH STACKS ARE FREE ##
#################################

basic(task_steps=check_yellow_side_stacks())

####################################
## ALTERNATIVE WHEN THERE IS NO 8 ##
####################################

basic(ID=81)

basic(m=Move.Distance(-300, 500, 500))

basic(m=Move.To(Area.BLUE_2.x, Area.BLUE_2.y + 180,
                  'f', 500, 1000, 10, 10),
      task_steps=two_level())
basic(m=Move.RotateTo(-1.57, 10, 10))

basic(task_steps=drop_two_level(-350))

basic(task_steps=check_yellow_side_stacks())

# basic(c=[Condition.Timeout(8, 0.01)])

basic(ID=82)

basic(m=Move.Distance(-300, 500, 500))

basic(m=Move.To(Area.BLUE_2.x, Area.BLUE_2.y + 250,
                  'f', 500, 1000, 10, 10),
      task_steps=two_level())

basic(m=Move.RotateTo(-1.57, 10, 10))

basic(task_steps=lift_two_on_one(250, back_distance=-500))

#################################
## CHECK WHICH STACKS ARE FREE ##
#################################

basic(c=[Condition.CheckStack('STACK9', 9)])

basic(task_steps=move_spline_stack9(1.57, det_ID=9))

basic(task_steps=pickup_front_full_stack(270, ID=9))

basic(m=Move.To(Area.BLUE_2.x,
                  Area.BLUE_2.y + 500,
                  'r', 1000, 500, 5, 5),
      task_steps=two_level())

basic(m=Move.RotateTo(-1.57, 15, 10))

basic(m=Move.Distance(350, 500, 500))
basic(task_steps=drop_two_level(back_distance=-500))

basic(ID=9)
basic(c=[Condition.CheckStack('STACK5', 5)])

basic(task_steps=move_spline_stack5(3.14-0.3535, det_ID=5))

basic(task_steps=pickup_front_full_stack(320, ID=5))

basic(m=Move.To(Area.BLUE_3.x + 60, Area.BLUE_3.y, 
'r', 1000, 1000, 10, 10),
      task_steps=two_level())

basic(m=Move.RotateTo(3.14, 10, 10))

basic(task_steps=drop_two_level(back_distance=-500))

basic(m=Move.RotateTo(-1.57, 15, 10),
      c=[Condition.MatchTime(99, 82),])


basic(ID=5,
      m=Move.RotateTo(3.14-0.3535, 15, 10))

basic(c=[Condition.CheckStack('STACK4', 4)])

basic(m=Move.To(700, 700, 'f', 1200, 1200, 15, 10))
basic(task_steps=move_to_front_STACK4())
# basic(task_steps=move_spline_stack4(None, y_off=-7.5, det_ID=4))

basic(task_steps=pickup_front_full_stack(ID=4))

basic(m=Move.Distance(-120, 500, 500))

basic(m=Move.RotateTo(-1.57, 5, 5),
      task_steps=two_level())

basic(m=Move.Distance(200, 500, 500))

basic(task_steps=drop_two_level(-270))

basic(m=Move.To(Area.BLUE_3.x + 500, 
                  Area.BLUE_3.y, 
                  'r', 1000, 500, 5, 5))

basic(m=Move.RotateTo(1.57, 15, 10),
      c=[Condition.MatchTime(99, 82),])

basic(ID=4) #,
      #m=Move.RotateTo(1.57, 15, 10))

basic(c=[Condition.CheckStack('STACK3', 100)]) 

basic(task_steps=move_to_front_STACK3())
# basic(task_steps=move_spline_stack3(1.57, y_off=15, det_ID=None))

basic(task_steps=pickup_front_full_stack(ID=100))

basic(m=Move.Distance(-600, 500, 500))

basic(m=Move.RotateTo(-3.14+0.707, 10, 10))

basic(m=Move.Distance(440, 500, 500),
      task_steps=two_level())

basic(task_steps=drop_two_level(-200))


basic(ID=99)
basic(m=Move.To(1500, 1000, 'f', 1000, 1000, 15, 10))

basic(c=[Condition.Timeout(100, 0.01),])

##########
## HOME ##
##########

basic(ID=100,
      m=Move.Distance(150, 1000, 1000),
      a=[I_O.Magnet(0)],
      s=[Servo.BackSwing(BackSwing.DROP),
         Servo.FrontCenterGrip(FrontCenterGripper.OPEN)])

basic(m=Move.To(Area.BLUE_HOME.x - 300,
                Area.BLUE_HOME.y - 800,
                'f', 1100, 1500, 15, 15),
      task_steps=init_all_servos())

# Wait for 99s to enter area
basic(c=[Condition.MatchTime(101, 98)])

basic(ID=101,
      m=Move.To(Area.BLUE_HOME.x - 300,
                Area.BLUE_HOME.y - 300,
                'f', 1100, 1500, 15, 15),
      p=10)
