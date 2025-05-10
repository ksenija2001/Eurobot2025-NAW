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

# TODO back more and close grippers beause of vertical projection
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
                     1000, 1500, 15, 10))

semi(m=Move.RotateTo(-1.57, 15, 10),
     task_steps=init_front_servos())

semi(task_steps=pickup_front_full_stack(400))

#################################
## LEAVE BANNER IN BLUE AREA 5 ##
#################################

semi(m=Move.Spline([Area.BLUE_5.x + 115],
                   [Area.BLUE_5.y + 175],
                   [1.57],
                   400,
                   'r'))

semi(task_steps=leave_banner())

#################################
## DROP STACK 5 IN BLUE AREA 3 ##
#################################

semi(m=Move.To(Area.BLUE_3.x + 100, 
               Area.BLUE_3.y - 25, 
               'f', 1100, 1000, 10, 5),
     task_steps=two_level())

semi(m=Move.RotateTo(3.14, 10, 5))

semi(m=Move.Distance(100, 1000, 500))
semi(task_steps=drop_two_level())

#####################
## PICK-UP STACK 3 ##
#####################

semi(m=Move.To(Area.BLUE_3.x + 175,
               MaterialStack.STACK3.y,
               'f', 1500, 1000, 15, 10),
     task_steps=init_back_servos())

semi(m=Move.RotateTo(3.14, 10, 5),
     s=[Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
         Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN),
         Servo.FrontVacuum(Vacuum.DOWN),
         Servo.CenterSwing(CenterSwing.DOWN, 50),
         Servo.CenterLift(CenterLift.DOWN),
         Servo.FrontGripLift(FrontGripLift.DOWN),
         Servo.FrontVacuumLift(VacuumLift.HOVER)])

semi(task_steps=pickup_front_full_stack())

semi(m=Move.Distance(-150, 500, 300))

semi(task_steps=two_level())
semi(task_steps=drop_one_level(p=-4))
semi(m=Move.RotateTo(0, 15, 15))
semi(task_steps=pickup_back_full_stack())

#########################################
## DROP HALF OF STACK 3 in BLUE AREA 3 ##
## AND LIFT OTHER HALF ON TWO LEVEL    ##
#########################################

semi(m=Move.Spline([Area.BLUE_3.x + 175],
                   [Area.BLUE_3.y - 25],
                   [3.14],
                   300,
                   'f')
)

semi(task_steps=lift_one_on_two(backout_distance=-200))

semi(m=Move.RotateTo(0.79, 5, 5),
     task_steps=init_front_servos())

semi(m=Move.Distance(100, 1000, 1000),
     task_steps=open_back())

#####################
## PICK-UP STACK 8 ##
#####################

semi(m=Move.Spline([3000-1850, MaterialStack.STACK8.x - 290],
                   [1250, MaterialStack.STACK8.y - 25],
                   [0.1, 0.0],
                   900,
                   'f'))

semi(task_steps=pickup_front_full_stack(200))

###############################
## PICK-UP STACK 1 WITH BACK ##
###############################

semi(m=Move.Spline([MaterialStack.STACK1.x],
                   [MaterialStack.STACK1.y - 325],
                   [-1.57],
                   500,
                   'r'),
     task_steps=init_back_servos())

semi(task_steps=pickup_back_full_stack(-250))

# semi(m=Move.Distance(300, 1500, 1500))

#################################
## DROP STACK 8 IN BLUE AREA 2 ##
#################################

# Moving through STACK 10
semi(m=Move.Spline([Area.BLUE_2.x - 20],
                   [Area.BLUE_2.y + 200],
                   [-1.57],
                   450,
                   'f'),
     s=[Servo.BackLift(BackGripLift.UP2)])

semi(m=Move.Distance(-100, 1000, 500),
     task_steps=two_level())

semi(task_steps=drop_two_level(-200))

semi(m=Move.RotateTo(0, 15, 10))

#################################
## PICK-UP STACK 6             ##
#################################

semi(m=Move.Spline([MaterialStack.STACK6.x],
                   [MaterialStack.STACK6.y + 250],
                   [-1.57],
                   500,
                   'f'),
     task_steps=init_front_servos())

semi(m=Move.Distance(100, 500, 500),
     c=[Condition.FrontSensors(5)])
semi(task_steps=pickup_front_full_stack(350))
semi(m=Move.Distance(-100, 1000, 500))

semi(task_steps=two_level())
semi(task_steps=drop_one_level(-250))

#################################
## LEAVE STACK 1 FORM BACK     ##
#################################

semi(m=Move.RotateTo(1.57, 5, 3),
     s=[Servo.BackLift(BackGripLift.DOWN)])

semi(m=Move.Distance(100, 500, 300),
     task_steps=open_back())

##########################################
## RETURN FOR LEFT STACK IN BLUE AREA 2 ##
##########################################

semi(m=Move.Spline([Area.BLUE_2.x - 25],
                   [Area.BLUE_2.y + 450],
                   [-1.57],
                   400,
                   'f'))

semi(task_steps=lift_one_on_two(200, backout_distance=-350))

################################################
## RETURN FOR LEFT STACK 1 IN FRONT OF AREA 4 ##
################################################

semi(m=Move.To(MaterialStack.STACK6.x - 20,
               MaterialStack.STACK6.y + 450,
               'f', 1000, 1000, 15, 15),
     task_steps=init_front_servos())

semi(m=Move.RotateTo(-1.57, 15, 10))

semi(task_steps=pickup_front_full_stack(300))
semi(task_steps=two_level())

semi(m=Move.Distance(100, 1000, 500),
     task_steps=drop_two_level())

# semi(task_steps=lift_two_on_one())

semi(c=[Condition.Timeout(100, 0.1),])

###########################
## END OF MAIN BRANCH    ##
###########################


##########################################
## ALTERNATIVE WHEN THERE IS NO STACK 5 ##
##########################################

semi(ID=5)

#################################
## LEAVE STACK 1 FROM BACK     ##
#################################
# Facing AREA 4
# semi(m=Move.Distance(-150, 1000, 500))

semi(m=Move.RotateTo(1.57, 3, 3),
     s=[Servo.BackLift(BackGripLift.DOWN)])

semi(m=Move.Distance(200, 500, 300),
     task_steps=open_back())

semi(m=Move.RotateTo(-1.57, 15, 10),
     task_steps=init_front_servos())

semi(task_steps=pickup_front_full_stack())

semi(task_steps=two_level())

semi(m=Move.Distance(200, 500, 500))
semi(task_steps=drop_one_level())

semi(m=Move.RotateTo(1.57, 15, 10))

##########################################
## RETURN FOR LEFT STACK IN BLUE AREA 2 ##
##########################################

semi(m=Move.Spline([Area.BLUE_2.x - 25 - 20],
                   [Area.BLUE_2.y + 450],
                   [-1.57],
                   400,
                   'f'))

semi(task_steps=lift_one_on_two(200))

semi(c=[Condition.Timeout(100, 0.1),])
# TODO CHECK TIME



##########
## HOME ##
##########

semi(ID=100,
      m=Move.Distance(150, 1000, 1000),
      task_steps=open_all())

semi(m=Move.To(Area.BLUE_HOME.x, 
                  Area.BLUE_HOME.y - 450, 
                  'f', 1100, 1500, 15, 15),
      task_steps=init_all_servos())

# Wait for 99s to enter area
semi(c=[Condition.MatchTime(101, 99)])

semi(ID=101,
      m=Move.To(Area.BLUE_HOME.x, 
                  Area.BLUE_HOME.y - 250, 
                  'f', 1100, 1500, 15, 15),
      p=10)