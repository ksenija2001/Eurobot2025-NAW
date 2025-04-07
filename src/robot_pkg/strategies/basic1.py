from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType

# Temporary
STARTING_X, STARTING_Y, STARTING_THETA = 0.0, 0.0, 90.0
STACK9_X, STACK9_Y, STACK9_THETA = 0.0, 0.0, 90.0
BACK_LIFT_BANNER, BACK_LIFT_DOWN, BACK_LIFT_UP = 0, 0, 0
FRONT_CENTER_GRIP_OPEN, FRONT_CENTER_GRIP_CLOSED = 0, 0
FRONT_SIDE_GRIP_OPEN, FRONT_SIDE_GRIP_CLOSED = 0, 0
FRONT_GRIP_LIFT_UP, FRONT_GRIP_LIFT_DOWN = 300, 0 # i guess
FRONT_VACUUM_DOWNWARD, FRONT_VACUUM_UPWARD = 60, 210 # ksenija rekla, odricem se odgovornosti

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
basic1(m=Move.To(STACK9_X, STACK9_Y - 70, STACK9_THETA, 800, 1000),
       s=[Servo.BackLift(BACK_LIFT_DOWN, 100),
       Servo.FrontCenterGrip(FRONT_CENTER_GRIP_OPEN, 100),
       Servo.FrontSideGrip(FRONT_SIDE_GRIP_OPEN, 100)]
       )

# Slowly move to pick up the stack
basic1(m=Move.Distance(70.0, 50.0, 500.0))

basic1(s=[Servo.FrontCenterGrip(FRONT_CENTER_GRIP_CLOSED, 50),
          Servo.FrontSideGrip(FRONT_SIDE_GRIP_CLOSED, 50)])

# Put the center swing on top of the planks and hold them in place + perhaps lift it all up a bit
basic1(s=Servo.FrontGripLift(FRONT_GRIP_LIFT_DOWN, 50))

# This is separate from the last step to prevent potential slipping of the plank
basic1(m = )

basic1()
