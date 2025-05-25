from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *
from robot_pkg.strategies.pickup_tasks import *
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
        sima1_coor=[Position(125,  1845, 0, 0),
                    Position(600, 1650, 0, 50),
                    Position(1000+100, 1500-50, 0, 50)],
        sima2_coor=[Position(125,  1725, 0, 0),
                    Position(1135, 1350, 0, 50),
                    Position(1475, 1400, 0, 50)],
        sima3_coor=[Position(125,  1605, 0, 0),
                    Position(1400, 1400, 0, 50),
                    Position(1850, 1450, 0, 50)],
        sima4_coor=[Position(125,  1905, 0, 0),
                    Position(1175, 1855, 0, 50),
                    Position(1300, 1300, 0, 30)])
)

visoka_gradnja(
    task_steps=init_position(
        Area.YELLOW_HOME.x - 77.5,
        Area.YELLOW_HOME.y - 48,
        3.14,
        'right corner')
)

######################
## PICK-UP STACK 9 ##
######################

visoka_gradnja(ID=1,
               m=Move.Spline([MaterialStack.STACK9.x + 10],
                             [MaterialStack.STACK9.y+350],
                             [0],
                             800,
                             'f'),
                task_steps=init_front_servos())

visoka_gradnja(m=Move.RotateTo(-1.57, 15, 10))

visoka_gradnja(task_steps=pickup_front_full_stack(375))

######################################
## SEPARATE STACK 9 IN YELLOW AREA 2 ##
######################################

visoka_gradnja(m=Move.Spline([Area.YELLOW_2.x + 45],
                             [Area.YELLOW_2.y + 110],  # 120
                             [-1.57],
                             400,
                             'f'),
               task_steps=two_level())

visoka_gradnja(task_steps=drop_separate_two_level(-275))

#####################
## PICK-UP STACK 5 ##
#####################

visoka_gradnja(m=Move.To(MaterialStack.STACK5.x + 10,
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

visoka_gradnja(m=Move.To(MaterialStack.STACK5.x - 50,
                         325,
                         'f', 1000, 600, 10, 5),
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

visoka_gradnja(m=Move.RotateTo(3.14, 15, 10))

visoka_gradnja(task_steps=pickup_front_full_stack(200))

# Make space for rotation while making two levels
visoka_gradnja(m=Move.Distance(-250, 1000, 1000))

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN YELLOW AREA 2 - SECOND TIME    ##
##############################################

visoka_gradnja(m=Move.To(MaterialStack.STACK5.x - 50,
                         325 + 10,   #- 20,
                         'f', 1200, 500, 10, 5),
               task_steps=two_level())

visoka_gradnja(m=Move.RotateTo(0.0, 5, 5))

visoka_gradnja(task_steps=lift_two_on_one(300, -135))
visoka_gradnja(task_steps=push_two_level(push_distance=135))

#####################
## PICK-UP STACK 2 ##
#####################

visoka_gradnja(m=Move.To(MaterialStack.STACK2.x + 25,
                         MaterialStack.STACK2.y - 350,
                         'f', 1000, 1000, 15, 15),
               task_steps=init_front_servos())

visoka_gradnja(m=Move.RotateTo(1.57, 15, 10))

visoka_gradnja(task_steps=pickup_front_full_stack(275))

###############################
## PICK-UP STACK 3 WITH BACK ##
###############################

visoka_gradnja(c=[Condition.CheckStack('STACK3', 3)])
visoka_gradnja(m=Move.Spline([MaterialStack.STACK3.x + 225],
                             [MaterialStack.STACK3.y + 10],
                             [0],
                             500,
                             'r'))

# visoka_gradnja(m=Move.RotateTo(0.0, 15, 5))
visoka_gradnja(task_steps=pickup_back_full_stack(back_distance=-150, forward_distance=400, ID=31))
# visoka_gradnja(m=Move.Distance(100, 500, 300))

##########################
## MOVE TO YELLOW AREA 2  ##
##########################

visoka_gradnja(m=Move.To(MaterialStack.STACK5.x - 50,
                         325 + 20,
                         'f', 1000, 500, 10, 5),
               task_steps=two_level())

visoka_gradnja(m=Move.RotateTo(0.0, 5, 5))

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN YELLOW AREA 2 - THIRD TIME     ##
##############################################

visoka_gradnja(task_steps=lift_two_on_one(280, -200))

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN YELLOW AREA 2 - FOURTH TIME      ##
##############################################


visoka_gradnja(m=Move.RotateTo(-1.57, 10, 5),
               task_steps=init_front_servos())

visoka_gradnja(m=Move.Distance(-100, 1000, 500))

# visoka_gradnja(m=Move.RotateTo(1.57, 5 ,5),
#             s=[Servo.BackLift(BackGripLift.DOWN)])

# visoka_gradnja(m=Move.Distance(200, 500, 500),
#                s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
#                   Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN)])
visoka_gradnja(task_steps=drop_back_one_level(1.57, 200))

visoka_gradnja(m=Move.RotateTo(-1.57, 10, 5))
visoka_gradnja(task_steps=pickup_front_full_stack(320))
visoka_gradnja(task_steps=two_level())
visoka_gradnja(task_steps=drop_two_level())


visoka_gradnja(c=[Condition.Timeout(100, 0.1),])

###############################################
## ID=8 ALTERNATIVE WHEN THERE IS NO STACK 3 ##
###############################################

visoka_gradnja(ID=3,
                m=Move.Distance(-300, 800, 500))

visoka_gradnja(ID=31,
                m=Move.Distance(400, 800, 500))

##########################
## MOVE TO YELLOW AREA 2  ##
##########################

visoka_gradnja(m=Move.To(MaterialStack.STACK5.x - 50,
                         325 + 20,
                         'f', 1000, 500, 10, 5),
               task_steps=two_level())

visoka_gradnja(m=Move.RotateTo(0, 5, 5))

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN YELLOW AREA 2 - THIRD TIME       ##
##############################################

visoka_gradnja(task_steps=lift_two_on_one(280, -250))

visoka_gradnja(m=Move.RotateTo(-1.57, 10, 5),
               task_steps=init_front_servos())
visoka_gradnja(m=Move.Distance(-500, 1000, 1000),
                c=[Condition.MatchTime(100, 82)])


#################################
## CHECK WHICH STACKS ARE FREE ##
#################################

visoka_gradnja(c=[Condition.CheckStack('STACK10', 10)])

visoka_gradnja(task_steps=move_spline_stack10(1.57, x_off=15, det_ID=10))

visoka_gradnja(task_steps=pickup_front_full_stack(270, ID=10))

visoka_gradnja(m=Move.To(Area.YELLOW_3.x - 100,
                  Area.YELLOW_3.y,
                  'f', 1000, 500, 5, 5),
      task_steps=two_level())

visoka_gradnja(m=Move.RotateTo(0.0, 15, 10))
visoka_gradnja(task_steps=drop_two_level())

visoka_gradnja(m=Move.Distance(-300, 1000, 1000),
                c=[Condition.MatchTime(99, 82),])


visoka_gradnja(ID=10)
visoka_gradnja(c=[Condition.CheckStack('STACK6', 6)])

visoka_gradnja(task_steps=move_spline_stack6(0.3535, x_off=10, det_ID=6))

visoka_gradnja(task_steps=pickup_front_full_stack(320, ID=6))

visoka_gradnja(m=Move.To(Area.YELLOW_3.x - 60, Area.YELLOW_3.y, 
'r', 1000, 1000, 10, 10),
      task_steps=two_level())

visoka_gradnja(m=Move.RotateTo(0.0, 10, 10))

visoka_gradnja(task_steps=drop_two_level(back_distance=-500))

visoka_gradnja(m=Move.RotateTo(-1.57, 15, 10), # rotation for when there is not skip
      c=[Condition.MatchTime(99, 82),])


visoka_gradnja(ID=6,
      m=Move.RotateTo(0.3535, 15, 10)) # rotation for when a skip occurs

visoka_gradnja(c=[Condition.CheckStack('STACK7', 7)])

visoka_gradnja(m=Move.To(3000-700, 700, 'f', 1200, 1200, 15, 10))
visoka_gradnja(task_steps=move_to_front_STACK7())
# visoka_gradnja(task_steps=move_spline_stack7(None, y_off=-15, det_ID=7))

visoka_gradnja(task_steps=pickup_front_full_stack(ID=7))

visoka_gradnja(m=Move.Distance(-120, 500, 500))

visoka_gradnja(m=Move.RotateTo(-1.57, 5, 5),
      task_steps=two_level())

visoka_gradnja(m=Move.Distance(200, 500, 500))

visoka_gradnja(task_steps=drop_two_level(-300))

visoka_gradnja(m=Move.To(Area.YELLOW_3.x - 500, 
                  Area.YELLOW_3.y, 
                  'r', 1000, 500, 5, 5))

visoka_gradnja(m=Move.RotateTo(1.57, 15, 10),
      c=[Condition.MatchTime(99, 82),])

visoka_gradnja(ID=7) #,
      #m=Move.RotateTo(1.57, 15, 10))

visoka_gradnja(c=[Condition.CheckStack('STACK8', 100)]) 
visoka_gradnja(task_steps=move_to_front_STACK8())
# visoka_gradnja(task_steps=move_spline_stack8(1.57, y_off=55, det_ID=None))

visoka_gradnja(task_steps=pickup_front_full_stack(ID=100))

visoka_gradnja(m=Move.Distance(-600, 500, 500))

visoka_gradnja(m=Move.RotateTo(-0.707, 10, 10))

visoka_gradnja(m=Move.Distance(440, 500, 500),
      task_steps=two_level())

visoka_gradnja(task_steps=drop_two_level(-200))


visoka_gradnja(ID=99)
visoka_gradnja(m=Move.To(1500, 1000, 'f', 1000, 1000, 15, 10))

visoka_gradnja(c=[Condition.Timeout(100, 0.01),])


##########
## HOME ##
##########

visoka_gradnja(ID=100,
      m=Move.Distance(150, 1000, 1000),
      a=[I_O.Magnet(0)],
      s=[Servo.BackSwing(BackSwing.DROP),
         Servo.FrontCenterGrip(FrontCenterGripper.OPEN)])

visoka_gradnja(m=Move.To(Area.YELLOW_HOME.x + 300,
                Area.YELLOW_HOME.y - 800,
                'f', 1100, 1500, 15, 15),
      task_steps=init_all_servos())

# Wait for 99s to enter area
visoka_gradnja(c=[Condition.MatchTime(101, 98)])

visoka_gradnja(ID=101,
      m=Move.To(Area.YELLOW_HOME.x + 300,
                Area.YELLOW_HOME.y - 300,
                'f', 1100, 1500, 15, 15),
      p=10)