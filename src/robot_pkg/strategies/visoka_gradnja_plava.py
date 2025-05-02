from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.play_elements import Area, MaterialStack
from robot_pkg.strategies.tasks import *

visoka_gradnja_plava = Strategy(color = Color.BLUE, 
                          square = Square.UPPER, 
                          mood = Mood.AGGRESSIVE)

#######################################
## STARTING POSITION: BLUE HOME AREA ##
#######################################

visoka_gradnja_plava(
    task_steps=init_position(
                            Area.BLUE_HOME.x - 77.5, 
                            Area.BLUE_HOME.y - 48, 
                            3.14, 
                            'left corner')
)

######################
## PICK-UP STACK 10 ##
######################

visoka_gradnja_plava(m=Move.Spline([MaterialStack.STACK10.x - 10],
                                    [MaterialStack.STACK10.y+350],
                                    [3.14],
                                    800,
                                    'f'),
                   task_steps=init_front_servos())

# visoka_gradnja_plava(m=Move.Distance(450, 1500, 1500))

# visoka_gradnja_plava(m=Move.To(MaterialStack.STACK10.x - 10, 
#                 MaterialStack.STACK10.y+350, 
#                 'f', 1500, 1500, 15, 10),
#       task_steps=init_front_servos())

visoka_gradnja_plava(m=Move.RotateTo(-1.57, 15, 10))

visoka_gradnja_plava(task_steps=pickup_front_full_stack(125))

######################################
## SEPARATE STACK 10 IN BLUE AREA 2 ##
######################################

visoka_gradnja_plava(m=Move.Spline([Area.BLUE_2.x - 50], 
                [Area.BLUE_2.y+80], 
                [-1.57],
                500,
                'f'),
    task_steps=two_level())
# visoka_gradnja_plava(m=Move.RotateTo(-1.57, 15, 10))

visoka_gradnja_plava(task_steps=drop_separate_two_level(-150))

#####################
## PICK-UP STACK 6 ##
#####################

visoka_gradnja_plava(m=Move.To(MaterialStack.STACK6.x, 
                MaterialStack.STACK6.y + 400,
                'f',
                1500, 1500, 15, 15),
     task_steps=init_front_servos())

visoka_gradnja_plava(m=Move.RotateTo(-1.57, 15, 10))

visoka_gradnja_plava(task_steps=pickup_front_full_stack(100))

#################################
## LEAVE BANNER IN BLUE AREA 4 ##
#################################

# Already in position
visoka_gradnja_plava(task_steps=leave_banner())

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN BLUE AREA 2                    ##
##############################################

visoka_gradnja_plava(m=Move.To(MaterialStack.STACK6.x+50, 
                325, 
                'f', 1000, 500, 10, 5))

visoka_gradnja_plava(m=Move.RotateTo(3.14, 10, 5),
    task_steps=two_level())

visoka_gradnja_plava(task_steps=lift_two_on_one(150, -100))
visoka_gradnja_plava(task_steps=push_two_level(0))

#####################
## PICK-UP STACK 7 ##
#####################

visoka_gradnja_plava(m=Move.To(MaterialStack.STACK7.x - 300, 
                MaterialStack.STACK7.y + 30, 
                'f', 1500, 1500, 15, 15),
      task_steps=init_front_servos())

visoka_gradnja_plava(m=Move.RotateTo(0, 15, 10))

visoka_gradnja_plava(task_steps=pickup_front_full_stack(-50))

# Make space for rotation while making two levels
visoka_gradnja_plava(m=Move.Distance(-250, 1000, 500))

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN BLUE AREA 2 - SECOND TIME      ##
##############################################

visoka_gradnja_plava(m=Move.To(MaterialStack.STACK6.x+50,  # ADD MORE BECAUSE THE WHOLE CONSTRUCTION MOVED
                325, 
                'f', 1500, 500, 10, 3),
    task_steps=two_level())

visoka_gradnja_plava(m=Move.RotateTo(3.14, 10, 5))

visoka_gradnja_plava(task_steps=lift_two_on_one(150, -100))
visoka_gradnja_plava(task_steps=push_two_level())

#####################
## PICK-UP STACK 1 ##
#####################

visoka_gradnja_plava(m=Move.To(MaterialStack.STACK1.x, 
                              MaterialStack.STACK1.y - 350, 
                              'f', 1000, 1500, 15, 15),
      task_steps=init_front_servos())

visoka_gradnja_plava(m=Move.RotateTo(1.57, 15, 3))

visoka_gradnja_plava(task_steps=pickup_front_full_stack())

# Make space for rotation while making two levels
# visoka_gradnja_plava(m=Move.Distance(-250, 300, 300))

###############################
## PICK-UP STACK 8 WITH BACK ##
###############################

visoka_gradnja_plava(m=Move.Spline([MaterialStack.STACK8.x - 225],
                 [MaterialStack.STACK8.y - 15],
                 [3.14],
                 450,
                 'r'))

visoka_gradnja_plava(m=Move.RotateTo(3.14, 15, 5))
visoka_gradnja_plava(task_steps=pickup_back_full_stack())
visoka_gradnja_plava(m=Move.Distance(100, 500, 300))

##########################
## MOVE TO BLUE AREA 2  ##
##########################

visoka_gradnja_plava(m=Move.To(MaterialStack.STACK6.x+50, 
                325, 
                'f', 1000, 500, 10, 5),   
      task_steps=two_level())

visoka_gradnja_plava(m=Move.RotateTo(3.14, 5, 5))

#################################
## LEAVE STACK 8 BEHIND ROBOT  ##
#################################

# visoka_gradnja_plava(m=Move.Distance(-200, 1200, 500),
#       s=[Servo.BackLift(BackGripLift.DOWN, 30)])
# visoka_gradnja_plava(m=Move.Distance(200, 300, 300),
#       s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
#         Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN)])

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN BLUE AREA 2 - THIRD TIME       ##
##############################################

visoka_gradnja_plava(task_steps=lift_two_on_one(150, 50))
# visoka_gradnja_plava(task_steps=push_two_level())

#######################################
## PICK-UP STACK 8 WHERE IT WAS LEFT ##
#######################################

# visoka_gradnja_plava(m=Move.RotateTo(0, 15, 10),
#       task_steps=init_front_servos())

# visoka_gradnja_plava(task_steps=pickup_front_full_stack())

##############################################
## LEAVE TWO LEVELS ON PREVIOUSLY SEPARATED ##
## STACKS IN BLUE AREA 2 - FOURTH TIME      ##
##############################################

# visoka_gradnja_plava(m=Move.RotateTo(-1.57, 10, 5),
#                      task_steps=two_level())
# visoka_gradnja_plava(task_steps=drop_two_level())

visoka_gradnja_plava(m=Move.RotateTo(1.57, 10, 5),
                        s=[Servo.BackLift(BackGripLift.DOWN, 30)],
                        task_steps=init_front_servos())

visoka_gradnja_plava(m=Move.Distance(250, 500, 500),
                     s=[Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
                        Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN)])

visoka_gradnja_plava(m=Move.RotateTo(-1.57, 15 ,10))
visoka_gradnja_plava(task_steps=pickup_front_full_stack())
visoka_gradnja_plava(task_steps=two_level())
visoka_gradnja_plava(task_steps=drop_two_level())

# visoka_gradnja_plava(m=Move.To(MaterialStack.STACK6.x+50, 
#                 325, 
#                 'f', 1000, 500, 10, 5),   
#       task_steps=two_level())

# visoka_gradnja_plava(m=Move.RotateTo(3.14, 10, 5))

# visoka_gradnja_plava(task_steps=lift_two_on_one(150, -100))
# visoka_gradnja_plava(task_steps=push_two_level())

##########
## HOME ##
##########

visoka_gradnja_plava(ID=100,
                  m=Move.Spline([Area.BLUE_HOME.x], 
                              [Area.BLUE_HOME.y-250], 
                              [-2.35],
                              800,
                              'r'))


# visoka_gradnja_plava(m=Move.Distance(200, 500, 500))

# visoka_gradnja_plava(task_steps=drop_one_level(-200))

# visoka_gradnja_plava(m=Move.To(Area.BLUE_2.x - 150, Area.BLUE_2.y+500, 'f', 1500, 1000, 15, 10))

# visoka_gradnja_plava(m=Move.RotateTo(-1.57, 15, 10))

# visoka_gradnja_plava(task_steps=lift_one_on_two(100))

# visoka_gradnja_plava(m=Move.RotateTo(3.14, 15, 15),
#     s=[Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
#          Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN),
#          Servo.FrontVacuum(Vacuum.DOWN),
#          Servo.FrontVacuumLift(VacuumLift.UP),
#          Servo.CenterSwing(CenterSwing.DOWN, 50),
#          Servo.CenterLift(CenterLift.DOWN),
#          Servo.FrontGripLift(FrontGripLift.DOWN)])


# visoka_gradnja_plava(task_steps=pickup_back_full_stack(+50))

# visoka_gradnja_plava(m=Move.Spline([MaterialStack.STACK7.x - 300],
#                     [MaterialStack.STACK7.y + 30],
#                     [0],
#                     350,
#                     'f'))

# visoka_gradnja_plava(task_steps=pickup_front_full_stack(-50))

# visoka_gradnja_plava(m=Move.Distance(-100, 300, 300),
#     task_steps=two_level())

# visoka_gradnja_plava(task_steps=drop_one_level(-100))

# visoka_gradnja_plava(m=Move.RotateTo(3.14, 5, 3),
#     s=[Servo.BackLift(BackGripLift.UP-20, 30)])

# visoka_gradnja_plava(m=Move.Distance(-300, 300, 300))
# visoka_gradnja_plava(s=[Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN),
#         Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
#         Servo.FrontSideGrip(FrontSideLeft.CLOSED, FrontSideRight.CLOSED),
#         Servo.BackLift(BackGripLift.UP-40)])

# visoka_gradnja_plava(m=Move.Distance(100, 300, 300))
# visoka_gradnja_plava(m=Move.To(Area.BLUE_2.x+400, Area.BLUE_2.y+50, 'f', 1500, 1000, 15, 10))

# visoka_gradnja_plava(m=Move.RotateTo(3.14, 10, 5))

# visoka_gradnja_plava(task_steps=lift_one_on_two())




