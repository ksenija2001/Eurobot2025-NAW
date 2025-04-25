from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
# from robot_pkg.play_elements import Area, MaterialStock
from robot_pkg.strategies.tasks import *

blue3 = Strategy(color = Color.BLUE, square = Square.LOWER, mood = Mood.PASSIVE)

blue3(m=Move.ResetOdom(1780, 230, 1.57))

blue3(task_steps=init_all_servos())
blue3(m=Move.Distance(200, 1500, 2000))
blue3(m=Move.To(2230, 580, 'f', 1500, 1500, 15, 10))
blue3(m=Move.RotateTo(-1.57, 5, 5))
# blue3(task_steps=pickup_back_full_stack())
blue3(task_steps=pickup_front_full_stack())
blue3(task_steps=two_level())
blue3(m=Move.Distance(100, 300, 300))
blue3(task_steps=drop_two_level())
