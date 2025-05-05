from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
# from robot_pkg.play_elements import Area, MaterialStock
from robot_pkg.strategies.tasks import *

strategy1 = Strategy(color=Color.YELLOW,
                     square=Square.LOWER, mood=Mood.PASSIVE)

strategy1(m=Move.ResetOdom(0, 0, 1.57),
          task_steps=init_front_servos())

strategy1(task_steps=pickup_front_full_stack())
strategy1(task_steps=two_level())

strategy1(task_steps=lift_two_on_one())
