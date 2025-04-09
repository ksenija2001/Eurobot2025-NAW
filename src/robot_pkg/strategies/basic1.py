from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType

# Temporary
STARTING_X, STARTING_Y, STARTING_THETA = 0.0, 0.0, 90.0
CONSTRUCTION2_X, CONSTRUCTION2_Y, CONSTRUCTION2_THETA = 0.0, 0.0, 0.0
CONSTRUCTION3_X, CONSTRUCTION3_Y, CONSTRUCTION3_THETA = 0.0, 0.0, 0.0
CONSTRUCTION4_X, CONSTRUCTION4_Y, CONSTRUCTION4_THETA = 0.0, 0.0, 0.0

STACK1_X, STACK1_Y, STACK1_THETA = 0.0, 0.0, -90.0
STACK3_X, STACK3_Y, STACK3_THETA = 0.0, 0.0, 90.0
STACK4_X, STACK4_Y, STACK4_THETA = 0.0, 0.0, -90.0
STACK9_X, STACK9_Y, STACK9_THETA = 0.0, 0.0, 90.0

BACK_LIFT_BANNER, BACK_LIFT_DOWN, BACK_LIFT_UP = 0, 0, 0
BACK_CENTER_GRIP_OPEN, BACK_CENTER_GRIP_CLOSED = 30, 50
BACK_SIDE_GRIP_OPEN, BACK_SIDE_GRIP_CLOSED = 40, 50
FRONT_CENTER_GRIP_OPEN, FRONT_CENTER_GRIP_CLOSED = 30, 50
FRONT_SIDE_GRIP_OPEN, FRONT_SIDE_GRIP_CLOSED = 30, 50
FRONT_GRIP_LIFT_UP, FRONT_GRIP_LIFT_DOWN = 250, 0
FRONT_VACUUM_DOWNWARD, FRONT_VACUUM_OUTSTRETCHED, FRONT_VACUUM_UPWARD_HOLDING, FRONT_VACUUM_UPWARD_PLACING = 60, 150, 230, 245
FRONT_VACUUM_LIFT_UP, FRONT_VACUUM_LIFT_PICKUP, FRONT_VACUUM_LIFT_PLANKS, FRONT_VACUUM_LIFT_DOWN = 300, 80, 200, 0  # Pickup - vacuum grippers on planks (on cans), planks - when planks are in the air
CENTER_SWING_UP, CENTER_SWING_DOWN = 240, 150
CENTER_LIFT_LEVEL3, CENTER_LIFT_LEVEL2, CENTER_LIFT_POSITIONING_CANS, CENTER_LIFT_DROPPING_CANS, CENTER_LIFT_DOWN = 300, 210, 160, 180, 0 # Dropping cans je kad ih fizički ostavi na dasci, positioning cans je kad ih pozicionira malo iznad daske

GRIPPER_CLOSED = 50

# Directions
FRONT = True
BACK = False

basic1 = Strategy(color = Color.YELLOW, square = Square.LOWER, mood = Mood.PASSIVE)

# Starting position DOWN, initialize servo motors, backLift in position for setting off the banner mechanism
basic1(m=Move.ResetOdom(STARTING_X, STARTING_Y, STARTING_THETA),
       s=[Servo.FrontSideGrip(30, 100),
        Servo.FrontCenterGrip(150, 100),
        Servo.FrontGripLift(0, 50),
        Servo.FrontVacuumLift(0, 10),
        Servo.FrontVacuum(60, 100),
        Servo.CenterSwing(150, 10),
        Servo.CenterLift(300, 10),
        Servo.BackLift(BACK_LIFT_BANNER, 100)],
        c = [(ConditionType.CINCH, 1)])

# A little rikverc
basic1(m=Move.Distance(-20.0, 100.0, 1000.0), ID=1)

# Can be removed if the back lift in the next step can be lowered later during the trajectory
basic1(m=Move.Distance(40.0, 100.0, 1000.0))

# First stack pickup (position 9, center)
basic1(m=Move.To(STACK9_X, STACK9_Y - 70.0, STACK9_THETA, 800, 1000, FRONT),
       s=[Servo.BackLift(BACK_LIFT_DOWN, 100),
       Servo.FrontCenterGrip(FRONT_CENTER_GRIP_OPEN, 100),
       Servo.FrontSideGrip(FRONT_SIDE_GRIP_OPEN, 100)])

# Slowly move to pick up the stack
basic1(m=Move.Distance(70.0, 100.0, 500.0))

# Close the grippers
basic1(s=[Servo.FrontCenterGrip(FRONT_CENTER_GRIP_CLOSED, 50),
          Servo.FrontSideGrip(FRONT_SIDE_GRIP_CLOSED, 50)])

# Put the center swing on top of the planks and hold them in place + perhaps lift it all up a bit
basic1(s=[Servo.FrontGripLift(FRONT_GRIP_LIFT_DOWN, 50)])

# This is separate from the last step to prevent potential slipping of the plank
# Move to the stack 4 position, open the gripper
basic1(m = Move.To(STACK4_X, STACK4_Y + 70.0, STACK4_THETA, 800, 1000, BACK),
       s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_OPEN, 100),
          Servo.BackSideGrip(BACK_SIDE_GRIP_OPEN, 100)])

# Slowly move to pick up the stack
basic1(m=Move.Distance(-70.0, 100.0, 500.0))

# Close the grippers
basic1(s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_CLOSED, 50),
          Servo.BackSideGrip(BACK_SIDE_GRIP_CLOSED, 50)])

# Move to the construction 2 position
basic1(m=Move.To(CONSTRUCTION2_X, CONSTRUCTION2_Y, CONSTRUCTION2_THETA, 100.0, 500.0, BACK))

# Open the grippers, leave the stack
basic1(s=[Servo.BackSideGrip(BACK_SIDE_GRIP_OPEN, 50), 
        Servo.BackCenterGrip(BACK_CENTER_GRIP_OPEN, 50)])

# Slowly rikverc a bit
basic1(m=Move.Distance(70.0, 100.0, 500.0))

# Move to the construction 3 position
basic1(m=Move.To(CONSTRUCTION3_X, CONSTRUCTION3_Y, CONSTRUCTION3_THETA, 100.0, 500.0, FRONT))

# basic1() SEQUENCE FOR CONSTRUCTING LEVEL 2 TRIBUNE

# Return to position 2 (the sequence finished with front grippers open)
basic1(m=Move.To(CONSTRUCTION2_X, CONSTRUCTION2_Y + 70.0, CONSTRUCTION2_THETA, 100.0, 500.0, FRONT))

# Go to the stack in position 2
basic1(m=Move.Distance(70.0, 100.0, 500.0))

# basic1() SEQUENCE FOR CONSTRUCTING LEVEL 2 TRIBUNE

# Go to stack 1
basic1(m=Move.To(STACK1_X, STACK1_Y - 70.0, STACK1_THETA, 800.0, 1000.0, FRONT),
       s=[Servo.FrontCenterGrip(FRONT_CENTER_GRIP_OPEN, 100),
          Servo.FrontSideGrip(FRONT_SIDE_GRIP_OPEN, 100)])

# Slowly move to pick up the stack
basic1(m=Move.Distance(70.0, 100.0, 500.0))

# Press the stack with vacuum grippers
basic1(s=[Servo.FrontCenterGrip(FRONT_CENTER_GRIP_CLOSED, 50),
          Servo.FrontSideGrip(FRONT_SIDE_GRIP_CLOSED, 50),
          Servo.FrontGripLift(FRONT_GRIP_LIFT_DOWN, 50),
          Servo.FrontVacuum(FRONT_VACUUM_DOWNWARD, 50)])

# Go to stack 3
basic1(m=Move.To(STACK3_X + 70.0, STACK3_Y, STACK3_THETA, 800, 1000, BACK),
       s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_OPEN, 100),
          Servo.BackSideGrip(BACK_SIDE_GRIP_OPEN, 100)])

# Slowly move to pick up the stack
basic1(m=Move.Distance(-70.0, 100.0, 500.0))

basic1(s=[Servo.BackCenterGrip(BACK_CENTER_GRIP_CLOSED, 50),
          Servo.BackSideGrip(BACK_SIDE_GRIP_CLOSED, 50)])

# Slowly moving to construction 3 because the back grippers with planks aren't secured, 
basic1(m=Move.To(CONSTRUCTION3_X, CONSTRUCTION3_Y, CONSTRUCTION3_THETA, 400.0, 200.0, FRONT))

