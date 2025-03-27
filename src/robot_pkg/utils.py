import time, datetime
import os, glob, subprocess, argparse
from threading import Event

from robot_pkg.step import Servo, ServoType, Move
from robot_pkg.consts import LOG_PATH, STRATEGIES_PATH
from robot_pkg.consts import IDs

# Command line auto complete for commands debugging
def completer(text, state):
    complete = [enum_item.name for enum_item in IDs]
    options = [cmd for cmd in complete if cmd.startswith(text)]
    if state < len(options):
        return options[state]
    else:
        return None

import readline
readline.parse_and_bind("tab: complete")
readline.set_completer(completer)

servo_dict = {
    ServoType.BACK_LEFT_LIFT.value: Servo.BackLeftLift,
    ServoType.BACK_RIGHT_LIFT.value: Servo.BackRightLift,
    ServoType.CENTER_LIFT.value: Servo.CenterLift,
    ServoType.CENTER_SWING.value: Servo.CenterSwing,
    ServoType.LEFT_GRIP_LIFT.value: Servo.LeftGripLift,
    ServoType.LEFT_VACUUM.value: Servo.LeftVacuum,
    ServoType.LEFT_VACUUM_LIFT.value: Servo.LeftVacuumLift,
    ServoType.RIGHT_GRIP_LIFT.value: Servo.RightGripLift,
    ServoType.RIGHT_VACUUM.value: Servo.RightVacuum,
    ServoType.RIGHT_VACUUM_LIFT.value: Servo.RightVacuumLift
}

def user_cmd(running:Event):
    while running.is_set():
        try:
            time.sleep(0.01)
            cmd = input("Message ID: ")
        except KeyboardInterrupt:
            break

        try:
            if IDs.has_key(cmd):
                msg_type = IDs[cmd].name

                if msg_type == IDs.SET_DISTANCE.name:
                    p = float(input("Target position: "))
                    v = float(input("Target velocity: "))
                    a = float(input("Target acceleratioon: "))
                    move = Move.Distance(p, v, a)
                    move._execute()

                elif msg_type == IDs.SET_MOTOR_SPEED.name:
                    left = (int)(input("Left motor velocity: "))
                    right = (int)(input("Right motor velocity: "))
                    move = Move.Speed(left, right)
                    move._execute()

                elif msg_type == IDs.SET_MOTOR_RPM.name:
                    left = (int)(input("Left motor RPM: "))
                    right = (int)(input("Right motor RPM: "))
                    move = Move.RPM(left, right)
                    move._execute()
                
                elif msg_type == IDs.SET_ROTATION_TO.name:
                    theta = float(input("Target angle[rad]: "))
                    w = float(input("Target angular velocity[rad/s]: "))
                    alpha = float(input("Target angular acceleration[rad/s^2]: "))
                    move = Move.RotateTo(theta, w, alpha)
                    move._execute()
                
                elif msg_type == IDs.SET_ROTATION_FOR.name:
                    theta = float(input("Target angle[rad]: "))
                    w = float(input("Target angular velocity[rad/s]: "))
                    alpha = float(input("Target angular acceleration[rad/s^2]: "))
                    move = Move.Rotate(theta, w, alpha)
                    move._execute()
                        
                # elif msg_type == IDs.RESET_ODOM.name:
                #     x = float(input("New x: "))
                #     y = float(input("New y: "))
                #     theta = float(input("New theta: "))
                #     data = struct.pack('3f', x, y, theta)
                #     can_handler.msg_send_queues[IDs[cmd].value].append(data)

                elif msg_type == IDs.SET_SERVO_POSITIONS.name:
    
                    print("Leave a field blank for exit")
                    while True:
                        id = input("ID: ")
                        if id == "":
                            break
                    
                        position = input("Position[degree]: ")
                        if position == "":
                            break
                        
                        speed = input("Speed[%]: ")
                        if speed == "":
                            break

                        servo = servo_dict[(int)(id)]((int)(position), (int)(speed))
                        servo._execute()
                    
                    Servo.send_positions()

                elif msg_type == IDs.GET_SERVO_POSITIONS.name:
                    id = input("ID: ")
                    Servo.check_position((int)(id))

                else:
                    print("Message ID is not of sending type")
                    continue                    
            else:
                print("Message ID doesn't exists")
        except KeyboardInterrupt:
            pass

def echo_log():
    # parser = argparse.ArgumentParser()
    LOG_TODAY = os.path.join(LOG_PATH, str(datetime.date.today()))
    latest_log = max(glob.glob(LOG_TODAY + "/*"), key=os.path.getctime)

    #parser.add_argument('filename', type=str, help='the name of the target')

    # parser.add_argument('log', type=str)

    # args = parser.parse_args()

    # if args.log == '':
    #     print("Specify file and log name")
    #     exit()

    # print(args.log)
    path = os.path.join(LOG_TODAY, latest_log)
    f = subprocess.Popen(['tail','-F', path],
            stdout=subprocess.PIPE,stderr=subprocess.PIPE)

    try:
        while True:
            print(f.stdout.readline())
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass

# from robot_pkg.old_strategy import Strategy
# from importlib import import_module
# def choose_strategy(color, square, mood):
#     temp_strategy = Strategy(color, square, mood)

#     for file in os.listdir(STRATEGIES_PATH):
#         if file.endswith(".py"):
#             strategy = os.path.splitext(file)[0]
#             mod = import_module("strategies." + strategy)
#             strategy = getattr(mod, strategy)

#             if strategy == temp_strategy:
#                 return strategy
    
#     print("STRATEGY NOT FOUND!!!!")
#     return None
