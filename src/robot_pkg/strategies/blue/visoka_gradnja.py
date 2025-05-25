from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *
from robot_pkg.strategies.pickup_tasks import *

###########################
## BLUE UPPER AGGRESSIVE ##
###########################

visoka_gradnja = Strategy(color=Color.BLUE,
                          square=Square.UPPER,
                          mood=Mood.AGGRESSIVE)

#################
## PREPARATION ##
#################

visoka_gradnja(
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

visoka_gradnja(
    task_steps=init_position(
        Area.BLUE_HOME.x - 77.5,
        Area.BLUE_HOME.y - 48,
        3.14,
        'left corner')
)

######################
## PICK-UP STACK 10 ##
######################

visoka_gradnja(ID=1,
               m=Move.Spline([MaterialStack.STACK10.x - 15],  # - 10],
                             [MaterialStack.STACK10.y+350],
                             [3.14],
                             800,
                             'f'),
                task_steps=init_front_servos())

visoka_gradnja(m=Move.RotateTo(-1.57, 15, 10))

visoka_gradnja(task_steps=pickup_front_full_stack(375))

######################################
## SEPARATE STACK 10 IN BLUE AREA 2 ##
######################################

visoka_gradnja(m=Move.Spline([Area.BLUE_2.x - 35],
                             [Area.BLUE_2.y + 110],
                             [-1.57],
                             400,
                             'f'),
               task_steps=two_level())

visoka_gradnja(task_steps=drop_separate_two_level(-275))

#####################
## PICK-UP STACK 6 ##
#####################

visoka_gradnja(m=Move.To(MaterialStack.STACK6.x - 20,
                         MaterialStack.STACK6.y + 400,
                         'f',
                         1500, 1500, 15, 15),
               task_steps=init_front_servos())

visoka_gradnja(m=Move.RotateTo(-1.57, 15, 5))

visoka_gradnja(task_steps=pickup_front_full_stack(360))

#################################
## LEAVE BANNER IN BLUE AREA 4 ##
#################################

# Already in position
visoka_gradnja(task_steps=leave_banner())

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN BLUE AREA 2                    ##
##############################################

visoka_gradnja(m=Move.To(MaterialStack.STACK6.x + 50,
                         325 - 10,
                         'f', 1000, 600, 10, 5) )

visoka_gradnja(m=Move.RotateTo(3.14, 5, 5),
               task_steps=two_level())

visoka_gradnja(task_steps=lift_two_on_one(300, -135))
visoka_gradnja(task_steps=push_two_level(push_distance=135))

#####################
## PICK-UP STACK 7 ##
#####################

visoka_gradnja(m=Move.To(MaterialStack.STACK7.x - 300,
                         MaterialStack.STACK7.y + 30,
                         'f', 1500, 1500, 15, 15),
               task_steps=init_front_servos())

visoka_gradnja(m=Move.RotateTo(0, 15, 10))

visoka_gradnja(task_steps=pickup_front_full_stack(200))

# Make space for rotation while making two levels
visoka_gradnja(m=Move.Distance(-250, 1000, 1000))

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN BLUE AREA 2 - SECOND TIME      ##
##############################################

visoka_gradnja(m=Move.To(MaterialStack.STACK6.x + 50,
                         325,
                         'f', 1200, 500, 10, 5),
               task_steps=two_level())

visoka_gradnja(m=Move.RotateTo(3.14, 5, 5))

visoka_gradnja(task_steps=lift_two_on_one(300, -135))
visoka_gradnja(task_steps=push_two_level(push_distance=135))

#####################
## PICK-UP STACK 1 ##
#####################

visoka_gradnja(m=Move.To(MaterialStack.STACK1.x - 10,
                         MaterialStack.STACK1.y - 350,
                         'f', 1000, 1000, 15, 15),
               task_steps=init_front_servos())

visoka_gradnja(m=Move.RotateTo(1.57, 15, 10))

visoka_gradnja(task_steps=pickup_front_full_stack(275))

###############################
## PICK-UP STACK 8 WITH BACK ##
###############################

visoka_gradnja(c=[Condition.CheckStack('STACK8', 8)])
visoka_gradnja(m=Move.Spline([MaterialStack.STACK8.x - 225],
                             [MaterialStack.STACK8.y + 10],
                             [3.14],
                             500,
                             'r'))

# visoka_gradnja(m=Move.RotateTo(3.14, 15, 5))
visoka_gradnja(task_steps=pickup_back_full_stack(forward_distance=400, ID=81))
# visoka_gradnja(m=Move.Distance(200, 1000, 500))

##########################
## MOVE TO BLUE AREA 2  ##
##########################

visoka_gradnja(m=Move.To(MaterialStack.STACK6.x + 50,
                         325 + 20,
                         'f', 1000, 500, 10, 5),
               task_steps=two_level())

visoka_gradnja(m=Move.RotateTo(3.14, 5, 5))

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN BLUE AREA 2 - THIRD TIME       ##
##############################################

visoka_gradnja(task_steps=lift_two_on_one(280, -200))

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN BLUE AREA 2 - FOURTH TIME      ##
##############################################

visoka_gradnja(m=Move.RotateTo(-1.57, 10, 5),
               task_steps=init_front_servos())
visoka_gradnja(m=Move.Distance(-100, 1000, 500))

# visoka_gradnja(m=Move.RotateTo(1.57, 5 ,5))

visoka_gradnja(task_steps=drop_back_one_level(1.57, 200))
# visoka_gradnja(m=Move.Distance(200, 500, 500),
#                s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
#                   Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN)])

visoka_gradnja(m=Move.RotateTo(-1.57, 10, 5))
visoka_gradnja(task_steps=pickup_front_full_stack(320))
visoka_gradnja(task_steps=two_level())
visoka_gradnja(task_steps=drop_two_level())
visoka_gradnja(c=[Condition.Timeout(100, 0.1),])


###############################################
## ID=8 ALTERNATIVE WHEN THERE IS NO STACK 8 ##
###############################################

visoka_gradnja(ID=8,
                m=Move.Distance(-300, 800, 500))

visoka_gradnja(ID=81,
                m=Move.Distance(400, 800, 500))
##########################
## MOVE TO BLUE AREA 2  ##
##########################

visoka_gradnja(m=Move.To(MaterialStack.STACK6.x + 50,
                         325 + 20,
                         'f', 1000, 500, 10, 5),
               task_steps=two_level())

visoka_gradnja(m=Move.RotateTo(3.14, 5, 5))

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN BLUE AREA 2 - THIRD TIME       ##
##############################################

visoka_gradnja(task_steps=lift_two_on_one(280, -250))

visoka_gradnja(m=Move.RotateTo(-1.57, 10, 5),
               task_steps=init_front_servos())
visoka_gradnja(m=Move.Distance(-500, 1000, 1000),
                c=[Condition.MatchTime(100, 82)])

#################################
## CHECK WHICH STACKS ARE FREE ##
#################################

visoka_gradnja(c=[Condition.CheckStack('STACK9', 9)])

visoka_gradnja(task_steps=move_spline_stack9(1.57, det_ID=9))

visoka_gradnja(task_steps=pickup_front_full_stack(270, ID=9))

visoka_gradnja(m=Move.To(Area.BLUE_3.x + 100,
                  Area.BLUE_3.y,
                  'f', 1000, 500, 5, 5),
      task_steps=two_level())

visoka_gradnja(m=Move.RotateTo(3.14, 15, 10))

visoka_gradnja(task_steps=drop_two_level())

visoka_gradnja(m=Move.Distance(-300, 1000, 1000),
            c=[Condition.MatchTime(99, 82),])

visoka_gradnja(ID=9)
visoka_gradnja(c=[Condition.CheckStack('STACK5', 5)])

visoka_gradnja(task_steps=move_spline_stack5(3.14-0.3535, det_ID=5))

visoka_gradnja(task_steps=pickup_front_full_stack(320, ID=5))

visoka_gradnja(m=Move.To(Area.BLUE_3.x + 60, Area.BLUE_3.y, 
'r', 1000, 1000, 10, 10),
      task_steps=two_level())

visoka_gradnja(m=Move.RotateTo(3.14, 10, 10))

visoka_gradnja(task_steps=drop_two_level(back_distance=-500))

visoka_gradnja(m=Move.RotateTo(-1.57, 15, 10),
      c=[Condition.MatchTime(99, 82),])


visoka_gradnja(ID=5,
      m=Move.RotateTo(3.14-0.3535, 15, 10))

visoka_gradnja(c=[Condition.CheckStack('STACK4', 4)])

visoka_gradnja(m=Move.To(700, 700, 'f', 1200, 1200, 15, 10))
visoka_gradnja(task_steps=move_to_front_STACK4())
# visoka_gradnja(task_steps=move_spline_stack4(None, y_off=-7.5, det_ID=4))

visoka_gradnja(task_steps=pickup_front_full_stack(ID=4))

visoka_gradnja(m=Move.Distance(-120, 500, 500))

visoka_gradnja(m=Move.RotateTo(-1.57, 5, 5),
      task_steps=two_level())

visoka_gradnja(m=Move.Distance(200, 500, 500))

visoka_gradnja(task_steps=drop_two_level(-270))

visoka_gradnja(m=Move.To(Area.BLUE_3.x + 500, 
                  Area.BLUE_3.y, 
                  'r', 1000, 500, 5, 5))

visoka_gradnja(m=Move.RotateTo(1.57, 15, 10),
      c=[Condition.MatchTime(99, 82),])

visoka_gradnja(ID=4) #,
      #m=Move.RotateTo(1.57, 15, 10))

visoka_gradnja(c=[Condition.CheckStack('STACK3', 100)]) 

visoka_gradnja(task_steps=move_to_front_STACK3())
# visoka_gradnja(task_steps=move_spline_stack3(1.57, y_off=15, det_ID=None))

visoka_gradnja(task_steps=pickup_front_full_stack(ID=100))

visoka_gradnja(m=Move.Distance(-600, 500, 500))

visoka_gradnja(m=Move.RotateTo(-3.14+0.707, 10, 10))

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

visoka_gradnja(m=Move.To(Area.BLUE_HOME.x - 300,
                Area.BLUE_HOME.y - 800,
                'f', 1100, 1500, 15, 15),
      task_steps=init_all_servos())

# Wait for 99s to enter area
visoka_gradnja(c=[Condition.MatchTime(101, 98)])

visoka_gradnja(ID=101,
      m=Move.To(Area.BLUE_HOME.x - 300,
                Area.BLUE_HOME.y - 300,
                'f', 1100, 1500, 15, 15),
      p=10)

