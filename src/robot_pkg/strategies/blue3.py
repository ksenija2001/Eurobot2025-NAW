from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
# from robot_pkg.play_elements import Area, MaterialStock
from robot_pkg.strategies.tasks import *

blue3 = Strategy(color = Color.BLUE, square = Square.UPPER, mood = Mood.AGGRESSIVE)

blue3(m=Move.ResetOdom(3000-150-225, 2000-230, -1.57),
        task_steps=init_all_servos())

blue3(m=Move.Distance(450, 1500, 1500))

blue3(m=Move.To(MaterialStack.STACK10.x - 20, MaterialStack.STACK10.y+350, 'f', 1500, 1500, 15, 10),
    s=[Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
         Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN),
         Servo.FrontVacuum(Vacuum.DOWN),
         Servo.CenterSwing(CenterSwing.DOWN, 50),
         Servo.CenterLift(CenterLift.DOWN),
         Servo.FrontGripLift(FrontGripLift.DOWN)])
blue3(m=Move.RotateTo(-1.57, 15, 5))

blue3(task_steps=pickup_front_full_stack(75))

blue3(m=Move.To(Area.BLUE_2.x - 50, Area.BLUE_2.y+80, 'f', 1500, 1000, 15, 5),
    task_steps=two_level())
blue3(m=Move.RotateTo(-1.57, 15, 5),
    s=[Servo.CenterLift(CenterLift.POSITION2+20)])

blue3(task_steps=drop_one_level(-150))

blue3(s=[Servo.CenterLift(CenterLift.DOWN),
        Servo.FrontVacuumLift(VacuumLift.PICKUP2+15, 20),
        Servo.FrontVacuum(Vacuum.DOWN, 25)])

blue3(s=[Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
        Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN)],
     a=[I_O.Pump(0), I_O.Valve(0)])

blue3(m=Move.Distance(-200, 300, 300),
    s=[Servo.FrontVacuumLift(VacuumLift.HOVER)])


blue3(m=Move.To(MaterialStack.STACK6.x, 
                MaterialStack.STACK6.y + 400,
                'f',
                1500, 1500, 15, 15),
       s=[Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
         Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN),
         Servo.FrontVacuum(Vacuum.DOWN),
         Servo.FrontVacuumLift(VacuumLift.UP),
         Servo.CenterSwing(CenterSwing.DOWN, 50),
         Servo.CenterLift(CenterLift.DOWN),
         Servo.FrontGripLift(FrontGripLift.DOWN)] )

blue3(m=Move.RotateTo(-1.57, 15, 10))

blue3(task_steps=pickup_front_full_stack(100))

blue3(m=Move.Rotate(3.14, 15, 10))


# LEAVING BANNER
blue3(m=Move.Distance(-250, 500, 500))

blue3(m=Move.Distance(125, 1500, 1500),
    s=[Servo.FrontVacuumLift(VacuumLift.PICKUP2)])

blue3(m=Move.To(MaterialStack.STACK6.x+50, 325, 'f', 1000, 500, 10, 5))

blue3(m=Move.RotateTo(3.14, 10, 5),
    task_steps=two_level())

blue3(task_steps=lift_two_on_one(150, -100))
blue3(s=[Servo.CenterLift(CenterLift.DOWN),
        Servo.FrontGripLift(FrontGripLift.DOWN),
        Servo.FrontVacuumLift(147)])

blue3(m=Move.Distance(150, 50, 50))

blue3(m=Move.Distance(-300, 1000, 1000))

blue3(m=Move.To(MaterialStack.STACK7.x - 300, MaterialStack.STACK7.y + 30, 'f', 1500, 1500, 15, 15),
    s=[Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
         Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN),
         Servo.FrontVacuum(Vacuum.DOWN),
         Servo.FrontVacuumLift(VacuumLift.UP),
         Servo.CenterSwing(CenterSwing.DOWN, 50),
         Servo.CenterLift(CenterLift.DOWN),
         Servo.FrontGripLift(FrontGripLift.DOWN)])

blue3(m=Move.RotateTo(0, 15, 10))

blue3(task_steps=pickup_front_full_stack(-50))

blue3(m=Move.Distance(-250, 300, 300))

blue3(m=Move.To(MaterialStack.STACK6.x+50, 325, 'f', 1000, 500, 10, 3),
    task_steps=two_level())

blue3(m=Move.RotateTo(3.14, 10, 5))

blue3(task_steps=lift_two_on_one(140, -100))
blue3(s=[Servo.CenterLift(CenterLift.DOWN),
        Servo.FrontGripLift(FrontGripLift.DOWN),
        Servo.FrontVacuumLift(147)])

blue3(m=Move.Distance(150, 50, 50))

blue3(m=Move.Distance(-300, 1000, 1000))

blue3(m=Move.To(MaterialStack.STACK1.x + 20, 
             MaterialStack.STACK1.y - 350, 
             'f', 2000, 2000, 15, 20),
  s=[Servo.FrontVacuumLift(VacuumLift.UP),
     Servo.CenterLift(CenterLift.DOWN),
     Servo.CenterSwing(CenterSwing.DOWN),
     Servo.FrontVacuum(Vacuum.DOWN),
     Servo.FrontGripLift(FrontGripLift.DOWN)])

blue3(m=Move.RotateTo(1.57, 15, 3))

blue3(task_steps=pickup_front_full_stack())

blue3(m=Move.Distance(-250, 300, 300))

blue3(m=Move.To(MaterialStack.STACK6.x+50, 325, 'f', 1000, 500, 10, 5))

blue3(m=Move.RotateTo(3.14, 10, 5),
    task_steps=two_level())

blue3(task_steps=lift_two_on_one(150, -100))
blue3(s=[Servo.CenterLift(CenterLift.DOWN),
        Servo.FrontGripLift(FrontGripLift.DOWN),
        Servo.FrontVacuumLift(147)])

blue3(m=Move.Distance(150, 50, 50))

blue3(m=Move.Distance(-300, 1000, 1000))


# blue3(m=Move.Distance(200, 500, 500))

# blue3(task_steps=drop_one_level(-200))

# blue3(m=Move.To(Area.BLUE_2.x - 150, Area.BLUE_2.y+500, 'f', 1500, 1000, 15, 10))

# blue3(m=Move.RotateTo(-1.57, 15, 10))

# blue3(task_steps=lift_one_on_two(100))

# blue3(m=Move.RotateTo(3.14, 15, 15),
#     s=[Servo.FrontCenterGrip(FrontCenterLeft.OPEN, FrontCenterRight.OPEN),
#          Servo.FrontSideGrip(FrontSideLeft.OPEN, FrontSideRight.OPEN),
#          Servo.FrontVacuum(Vacuum.DOWN),
#          Servo.FrontVacuumLift(VacuumLift.UP),
#          Servo.CenterSwing(CenterSwing.DOWN, 50),
#          Servo.CenterLift(CenterLift.DOWN),
#          Servo.FrontGripLift(FrontGripLift.DOWN)])


# blue3(task_steps=pickup_back_full_stack(+50))

# blue3(m=Move.Spline([MaterialStack.STACK7.x - 300],
#                     [MaterialStack.STACK7.y + 30],
#                     [0],
#                     350,
#                     'f'))

# blue3(task_steps=pickup_front_full_stack(-50))

# blue3(m=Move.Distance(-100, 300, 300),
#     task_steps=two_level())

# blue3(task_steps=drop_one_level(-100))

# blue3(m=Move.RotateTo(3.14, 5, 3),
#     s=[Servo.BackLift(BackGripLift.UP-20, 30)])

# blue3(m=Move.Distance(-300, 300, 300))
# blue3(s=[Servo.BackCenterGrip(BackCenterLeft.OPEN, BackCenterRight.OPEN),
#         Servo.BackSideGrip(BackSideLeft.OPEN, BackSideRight.OPEN),
#         Servo.FrontSideGrip(FrontSideLeft.CLOSED, FrontSideRight.CLOSED),
#         Servo.BackLift(BackGripLift.UP-40)])

# blue3(m=Move.Distance(100, 300, 300))
# blue3(m=Move.To(Area.BLUE_2.x+400, Area.BLUE_2.y+50, 'f', 1500, 1000, 15, 10))

# blue3(m=Move.RotateTo(3.14, 10, 5))

# blue3(task_steps=lift_one_on_two())




