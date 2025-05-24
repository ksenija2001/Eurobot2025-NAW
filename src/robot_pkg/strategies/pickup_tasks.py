from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move
from robot_pkg.opponent import Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
from robot_pkg.misc import *
from robot_pkg.play_elements import *
from robot_pkg.consts import Points
from robot_pkg.strategies.tasks import *
import math

def move_to_front_STACK7(init=True):
    s = Strategy()

    if init:
        s(m=Move.To(MaterialStack.STACK7.x - 290,
                    MaterialStack.STACK7.y - 20,
                    'f', 1500, 1500, 15, 15),
        task_steps=init_front_servos())

        s(m=Move.RotateTo(0, 15, 10))
    else:
        s(m=Move.To(MaterialStack.STACK7.x - 290,
            MaterialStack.STACK7.y - 20,
            'f', 1000, 1000, 10, 5))
        
        s(m=Move.RotateTo(0, 5, 5))

    return s.steps

def move_to_front_STACK6(init=True):
    s = Strategy()

    if init:
        s(m=Move.To(MaterialStack.STACK6.x + 10,
                    MaterialStack.STACK6.y + 200,
                    'f', 1500, 1500, 15, 15),
         task_steps=init_front_servos())

        s(m=Move.RotateTo(-1.57, 15, 10))

    else:
        s(m=Move.To(MaterialStack.STACK6.x + 10,
                    MaterialStack.STACK6.y + 200,
                    'f', 1000, 1000, 10, 5))

        s(m=Move.RotateTo(-1.57, 5, 5))

    return s.steps

def move_to_front_STACK10(init=True):
    s = Strategy()

    if init:
        s(m=Move.To(MaterialStack.STACK10.x + 10,
                    MaterialStack.STACK10.y - 350,
                    'f', 1500, 1500, 15, 15),
        task_steps=init_front_servos())

        s(m=Move.RotateTo(1.57, 15, 10))
    else:
        s(m=Move.To(MaterialStack.STACK10.x + 10,
            MaterialStack.STACK10.y - 350,
            'f', 1000, 1000, 10, 5))

        s(m=Move.RotateTo(1.57, 5, 5))

    return s.steps

def move_to_front_STACK1(init=True):
    s = Strategy()

    if init:
        s(m=Move.To(MaterialStack.STACK1.x - 10,
                    MaterialStack.STACK1.y - 365,
                    'f', 1500, 1500, 15, 15),
        task_steps=init_front_servos())

        s(m=Move.RotateTo(1.57, 15, 10))
    else:
        s(m=Move.To(MaterialStack.STACK1.x - 10,
                    MaterialStack.STACK1.y - 365,
                    'f', 1000, 1000, 10, 5))

        s(m=Move.RotateTo(1.57, 5, 5))

    return s.steps