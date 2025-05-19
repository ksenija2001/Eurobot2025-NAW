from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *

########################
## BLUELOWER PASSIVE FOR HOMOLOGATION ##
########################

homologue = Strategy(color=Color.BLUE, square=Square.LOWER, mood=Mood.HOMO)

#################
## PREPARATION ##
#################

# homologue(
#     task_steps=sima_coordinates(
#         sima1_coor=[Position(125, -1605, 0, 0),
#                     Position(1000, -1300, 0, 50),
#                     Position(1800, -1400, 0, 50)],
#         sima2_coor=[Position(125, -1724, 0, 0),
#                     Position(300, -1724, 0, 50),
#                     Position(900, -1400, 0, 50),
#                     Position(1300, -1360, 0, 50)],
#         sima3_coor=[Position(125, -1815, 0, 0),
#                     Position(500, -1815, 0, 50),
#                     Position(900, -1500, 0, 50)],
#         sima4_coor=[Position(125, -1915, 0, 0),
#                     Position(1300, -1915, 0, 20),
#                     Position(1300, -1415, 0, 4)])
# )

# homologue(task_steps=init_position(
#     Area.BLUE_2.x + 77.5,
#     Area.BLUE_2.y + 48,
#     0.0,
#     'middle'))

# ###################################
# ## LEAVING BANNER IN BLUEAREA 2 ##
# ###################################

# homologue(ID=1,
#       task_steps=leave_banner())

# ######################
# ## PICK-UP STACK 9 ##
# ######################

# homologue(m=Move.To(MaterialStack.STACK10.x,
#                 MaterialStack.STACK10.y - 350,
#                 'f', 1000, 1000, 10, 10), 
#       task_steps=init_front_servos())

# homologue(m=Move.RotateTo(1.57, 10, 10))  

# homologue(task_steps=(pickup_front_full_stack(330)))

# homologue(m=Move.To(Area.BLUE_2.x,
#                 Area.BLUE_2.y + 50,
#                 'f', 1000, 500, 10, 5))

# homologue(task_steps=two_level())

# homologue(task_steps=drop_two_level())

# ##########
# ## HOME #
# ##########
# homologue(ID=100,
#       m=Move.Distance(150, 1000, 1000),
#       task_steps=open_all())

# homologue(m=Move.To(Area.YELLOW_HOME.x+300, Area.YELLOW_HOME.y - 500, 'f', 1000, 1000, 10, 10),
#       task_steps=init_all_servos())

# # Wait for 99s to enter area
# homologue(c=[Condition.MatchTime(101, 99)])


# homologue(ID=101,
#       m=Move.To(Area.YELLOW_HOME.x+300, 
#                   Area.YELLOW_HOME.y - 300, 
#                   'f', 1000, 1000, 10, 10),
#       p=10)
