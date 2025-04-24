from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
# from robot_pkg.play_elements import Area, MaterialStock
from robot_pkg.strategies.tasks import *

strategy1 = Strategy(color = Color.BLUE, square = Square.LOWER, mood = Mood.PASSIVE)

strategy1(m=Move.ResetOdom(1780, 230, 1.57))

strategy1(task_steps=init_all_servos())
strategy1(m=Move.Distance(200, 1500, 2000))
strategy1(m=Move.To(2230, 580, 'f', 1500, 1500, 15, 10))
strategy1(m=Move.RotateTo(-1.57, 5, 5))
# strategy1(task_steps=pickup_back_full_stack())
strategy1(task_steps=pickup_front_full_stack())
strategy1(task_steps=two_level())
strategy1(m=Move.Distance(100, 300, 300))
strategy1(task_steps=drop_two_level())
