from robot_pkg.strategy import Strategy, Color, Square, Mood
from robot_pkg.move import Move, Position
from robot_pkg.servo import Servo
from robot_pkg.in_out import I_O
from robot_pkg.conditions import ConditionType, Condition
# from robot_pkg.play_elements import Area, MaterialStock
from robot_pkg.strategies.tasks import *

kula_od_karata = Strategy(color = Color.BLUE, square = Square.LOWER, mood = Mood.PASSIVE)

kula_od_karata(m=Move.ResetOdom(Area.BLUE_2.x, Area.BLUE_2.y, 1.57))

kula_od_karata(task_steps=init_all_servos())

kula_od_karata(s=[Servo.FrontCenterGrip(Gripper.OPEN),
                  Servo.FrontSideGrip(SideGripper.OPEN),
                  Servo.CenterSwing(CenterSwing.DOWN)])

kula_od_karata(m=Move.Spline([MaterialStack.STACK10.x, MaterialStack.STACK9.x-20], 
                        [MaterialStack.STACK10.y-100, MaterialStack.STACK9.y+300], 
                        [1.57, 3.14], 
                        400, 'f'))

kula_od_karata(s=[Servo.FrontCenterGrip(Gripper.GRIP),
                  Servo.FrontSideGrip(SideGripper.GRIP),
                  Servo.FrontVacuumLift(VacuumLift.PICKUP2+30),
                  Servo.FrontGripLift(FrontGripLift.HOVER),
                  Servo.CenterLift(CenterLift.HOVER)])

kula_od_karata(m=Move.RotateTo(1.57, 15, 5),
               s=[Servo.BackCenterGrip(Gripper.OPEN),
                  Servo.BackSideGrip(Gripper.OPEN)])

kula_od_karata(m=Move.Distance(-350, 300, 500),
               s=[Servo.BackCenterGrip(Gripper.GRIP, activate_pose=Position(3000, MaterialStack.STACK9.y-70)),
                  Servo.BackSideGrip(Gripper.GRIP, activate_pose=Position(3000, MaterialStack.STACK9.y-70))])

kula_od_karata(m=Move.To(Area.BLUE_3.x+250, Area.BLUE_3.y, 'f', 1500, 1000, 15, 5))
kula_od_karata(m=Move.RotateTo(3.14, 15, 5))

kula_od_karata(s=[Servo.FrontVacuumLift(VacuumLift.PICKUP2)],
                a=[I_O.Pump(1), I_O.Valve(1)])

kula_od_karata(task_steps=two_level())
kula_od_karata(task_steps=drop_two_level())

kula_od_karata(m=Move.RotateTo(1.57, 15, 5),
              s=[Servo.FrontVacuumLift(VacuumLift.UP),
                Servo.FrontVacuum(Vacuum.DOWN),
                Servo.CenterLift(CenterLift.DOWN)])
kula_od_karata(s=[Servo.BackCenterGrip(Gripper.OPEN),
                  Servo.BackSideGrip(Gripper.OPEN)])

kula_od_karata(m=Move.Distance(200, 300, 300))
kula_od_karata(m=Move.RotateTo(-1.57, 15, 10))
kula_od_karata(task_steps=pickup_front_full_stack())
kula_od_karata(task_steps=two_level())
kula_od_karata(task_steps=drop_one_level())

kula_od_karata(m=Move.To(Area.BLUE_3.x+200, Area.BLUE_3.y+500, 'r', 1500, 1000, 15, 5))

kula_od_karata(m=Move.RotateTo(-1.57, 15, 10))

# kula_od_karata(m=Move.Distance(150, 300, 300))
kula_od_karata(task_steps=lift_one_on_two())


