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

def move_spline_stack9(rotation = 0, det_ID=None):
    s = Strategy()

    if rotation != 0:
        s(m=Move.RotateTo(rotation, 15, 10))

    s(m=Move.Spline([MaterialStack.STACK9.x - 10],
                        [MaterialStack.STACK9.y + 300],
                        [3.14],
                        500, 'f'),
        task_steps=init_front_servos(),
        c=[Condition.Detection(det_ID, 0)])
    
    s(m=Move.RotateTo(-1.57, 15, 10))

    return s.steps


def move_spline_stack5(rotation = 0, det_ID=None):
    s = Strategy()

    if rotation != 0:
        s(m=Move.RotateTo(rotation, 15, 10))

    s(m=Move.Spline([MaterialStack.STACK5.x],
                        [MaterialStack.STACK5.y + 340],
                        [-1.57],
                        550, 'f'),
        task_steps=init_front_servos(),
        c=[Condition.Detection(det_ID, 0)])

    return s.steps

def move_spline_stack4(rotation = 0, x_off=0, y_off=0, det_ID=None):
    s = Strategy()

    if rotation != 0:
        s(m=Move.RotateTo(rotation, 15, 10))

    s(m=Move.Spline([MaterialStack.STACK4.x + 350 + x_off],
                        [MaterialStack.STACK4.y + y_off],
                        [3.14],
                        550, 'f'),
        task_steps=init_front_servos(),
        c=[Condition.Detection(det_ID, 0)])

    return s.steps

def move_spline_stack3(rotation = 0, x_off=0, y_off=0, det_ID=None):
    s = Strategy()

    if rotation != 0:
        s(m=Move.RotateTo(rotation, 15, 10))

    s(m=Move.Spline([MaterialStack.STACK3.x + 350 + x_off],
                        [MaterialStack.STACK3.y - 50 + y_off],
                        [3.14],
                        550, 'f'),
        task_steps=init_front_servos(),
        c=[Condition.Detection(det_ID, 0)])

    return s.steps


def check_yellow_side_stacks():
    s = Strategy()

    s(c=[Condition.CheckStack('STACK9', 9)])

    s(task_steps=move_spline_stack9(1.57, det_ID=9))
    # s(m=Move.RotateTo(1.57, 15, 10))
    # s(m=Move.Spline([MaterialStack.STACK9.x],
    #                     [MaterialStack.STACK9.y + 300],
    #                     [3.14],
    #                     500, 'f'),
    #     task_steps=init_front_servos(),
    #     c=[Condition.Detection(9, 0)])

    # s(m=Move.RotateTo(-1.57, 15, 10))

    s(task_steps=pickup_front_full_stack(270, ID=9))

    s(m=Move.To(Area.BLUE_2.x,
                    Area.BLUE_2.y + 500,
                    'r', 1000, 500, 5, 5),
        task_steps=two_level())

    s(m=Move.RotateTo(1.57, 15, 10))

    s(task_steps=drop_one_level(p=-4))

    s(m=Move.RotateTo(-1.57, 15, 10),
        task_steps=init_back_servos())

    s(task_steps=pickup_back_full_stack())

    s(task_steps=lift_one_on_two(forward_distance=280, backout_distance=-200))

    s(task_steps=drop_back_one_level(rotation=1.57, forward_distance=-100))

    s(m=Move.Distance(100, 500, 500),
        c=[Condition.InPosition(100),])


    s(ID=9)
    s(c=[Condition.CheckStack('STACK5', 5)])

    s(task_steps=move_spline_stack5(3.14-0.3535, det_ID=5))
    # s(m=Move.RotateTo(3.14-0.3535, 15, 10))
    # s(m=Move.Spline([MaterialStack.STACK5.x + 10],
    #                     [MaterialStack.STACK5.y + 340],
    #                     [-1.57],
    #                     550, 'f'),
    #     task_steps=init_front_servos(),
    #     c=[Condition.Detection(5, 0)])

    s(task_steps=pickup_front_full_stack(320, ID=5))

    s(m=Move.To(Area.BLUE_3.x + 300, Area.BLUE_3.y, 'r', 1000, 500, 5, 5),
        task_steps=two_level())

    s(m=Move.RotateTo(3.14, 10, 10))

    s(task_steps=drop_one_level(-100))

    s(m=Move.To(Area.BLUE_2.x,
                    Area.BLUE_2.y + 500,
                    'r', 1500, 1000, 15, 10))

    s(m=Move.RotateTo(-1.57, 15, 10))

    s(task_steps=lift_one_on_two(340))

    s(c=[Condition.Timeout(100, 0.01),])


    s(ID=5)
    s(c=[Condition.CheckStack('STACK4', 4)])

    s(task_steps=move_spline_stack4(3.14-0.3535, det_ID=4))
    # s(m=Move.RotateTo(3.14-0.3535, 15, 10))
    # s(m=Move.Spline([MaterialStack.STACK4.x + 350],
    #                     [MaterialStack.STACK4.y],
    #                     [3.14],
    #                     550, 'f'),
    #     task_steps=init_front_servos(),
    #     c=[Condition.Detection(4, 0)])

    s(task_steps=pickup_front_full_stack(ID=4))

    s(m=Move.Distance(-120, 500, 500))

    s(m=Move.RotateTo(-1.57, 5, 5),
        task_steps=two_level())

    s(m=Move.Distance(200, 500, 500))

    s(task_steps=drop_one_level(-600))

    s(m=Move.To(Area.BLUE_2.x,
                    Area.BLUE_2.y + 500,
                    'f', 1500, 1000, 15, 10))

    s(m=Move.RotateTo(-1.57, 15, 10))
    s(task_steps=lift_one_on_two(320))

    s(c=[Condition.Timeout(100, 0.01),])


    s(ID=4)
    s(c=[Condition.CheckStack('STACK3', 100)]) 

    s(task_steps=move_spline_stack3(3.14-0.3535, det_ID=None))
    # s(m=Move.RotateTo(3.14-0.3535, 15, 10))
    # s(m=Move.Spline([MaterialStack.STACK3.x + 350],
    #                     [MaterialStack.STACK3.y - 50],
    #                     [3.14],
    #                     550, 'f'),
    #     task_steps=init_front_servos())

    s(task_steps=pickup_front_full_stack(ID=100))

    s(m=Move.Distance(-120, 500, 500))

    s(m=Move.RotateTo(-1.57, 10, 10), )

    s(m=Move.Distance(400, 500, 500),
        task_steps=two_level())

    s(task_steps=drop_one_level(-200))

    s(m=Move.To(Area.BLUE_2.x,
                    Area.BLUE_2.y + 400,
                    'f', 1500, 1000, 15, 10))

    s(m=Move.RotateTo(-1.57, 15, 10))

    s(task_steps=lift_one_on_two(320))

    s(c=[Condition.Timeout(100, 0.01),])

    return s.steps
