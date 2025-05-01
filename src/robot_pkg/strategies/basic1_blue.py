from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType

from robot_pkg.misc import *
from robot_pkg.strategies.tasks import *

FRONT = True
BACK = False


# ovo sigurno moze i automatski da se radi spram odabira boje (posotoji da bi strategija mogla samo da se prekopira na drugu boju)
# STARTING1_X, STARTING1_Y, STARTING1_THETA = STARTING1_BLUE_X, STARTING1_BLUE_Y, STARTING1_BLUE_THETA
# STARTING2_X, STARTING2_Y, STARTING2_THETA = STARTING2_BLUE_X, STARTING2_BLUE_Y, STARTING2_BLUE_THETA
# STARTING3_X, STARTING3_Y, STARTING3_THETA = STARTING3_BLUE_X, STARTING3_BLUE_Y, STARTING3_BLUE_THETA
# FIELD1_X, FIELD1_Y, FIELD1_THETA = FIELD1_BLUE_X, FIELD1_BLUE_Y, FIELD1_BLUE_THETA
# FIELD2_X, FIELD2_Y, FIELD2_THETA = FIELD2_BLUE_X, FIELD2_BLUE_Y, FIELD2_BLUE_THETA
# FIELD3_X, FIELD3_Y, FIELD3_THETA = FIELD3_BLUE_X, FIELD3_BLUE_Y, FIELD3_BLUE_THETA
# FIELD4_X, FIELD4_Y, FIELD4_THETA = FIELD4_BLUE_X, FIELD4_BLUE_Y, FIELD4_BLUE_THETA
# FIELD5_X, FIELD5_Y, FIELD5_THETA = FIELD5_BLUE_X, FIELD5_BLUE_Y, FIELD5_BLUE_THETA

basic1_blue = Strategy(color = Color.BLUE, square = Square.UPPER, mood = Mood.PASSIVE)

# # Starting position DOWN, initialize servo motors, backLift in position for setting off the banner mechanism
# basic1_blue(m=Move.ResetOdom(STARTING2_X, STARTING2_Y, STARTING2_THETA), # TODO: define the starting angles (lower square)
#        s=[init_front_servos(),
#         init_back_servos()],
#         c = [(ConditionType.CINCH, 1)])

# # Banner
# basic1_blue(m=Move.Distance(-20.0, 100.0, 1000.0), ID=1)

# # Small slow forward motion to leave the banner properly
# basic1_blue(m=Move.Distance(50.0, 100.0, 1000.0))

# # Go to the stack in position 1
# basic1_blue(pickup_front_regular(STACK10_X, STACK10_Y, STACK10_THETA, 0, 0, 0)) #swap the dummy constants with proper coordinates

# # Go to the stack in position 3
# basic1_blue(pickup_back_regular(STACK6_X, STACK6_Y, STACK6_THETA, 0, 0, 0)) #swap the dummy constants with proper coordinates

# # Continue forward to leave the stack in position 4
# basic1_blue(m=Move.Distance(-50.0, 100.0, 1000.0))

# basic1_blue(ungrip_back())

# basic1_blue(m=Move.Distance(50.0, 100.0, 1000.0))

# # insert spline here

# basic1_blue(m=Move.To(STARTING2_X, STARTING2_Y, STARTING2_THETA, FRONT))

# basic1_blue(two_level())

# basic1_blue(pickup_front_regular(STACK7_X, STACK7_Y, STACK7_THETA, 0, 0, 0))

# basic1_blue(m=Move.To(STARTING2_X, STARTING2_Y, STARTING2_THETA, FRONT))

# # lift the picked up stack to the existing 2 level on field 2
# basic1_blue(lift_third_level()) # TODO: needs to be implemented (vidim da vec postoji funkcija koja stavlja na treci nivo ali ne mogu da razaznam sta je sta)

# basic1_blue(m=Move.Distance(FIELD4_X - 150.0, FIELD4_Y - 150.0, FIELD4_THETA, 200.0, 1000.0, FRONT))

# basic1_blue(ungrip_front())

# # the stack previously left at field 4 is made into a two level tribune
# basic1_blue(two_level())

# basic1_blue(pickup_front_regular(STACK1_X, STACK1_Y, STACK1_THETA, 0, 0, 0))

# basic1_blue(pickup_front_two_level(STACK8_X, STACK8_Y, STACK8_THETA, 0, 0, 0))

# # also putting it in starting position 2 so it doeesn't be in the way of SIMA
# basic1_blue(m=Move.To(STARTING2_X, STARTING2_Y, STARTING2_THETA, FRONT)) # offsets needed
# basic1_blue(ungrip_front)

# basic1_blue(m=Move.To(HOME_X, HOME_Y, HOME_THETA, BACK))