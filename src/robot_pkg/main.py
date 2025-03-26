
from threading import Event, Thread
# Global access variables
paused:Event = Event()
import time, os, sys
import struct
import math
import readline
from importlib import import_module

from robot_pkg.old_strategy import Strategy

from robot_pkg.logger import LogHandler
log_handler = LogHandler()

from robot_pkg.can_controller import CanNetwork
can_handler = CanNetwork(channel='can0', interface='socketcan', max_queue_size=10)
can_handler.init_queues(10)

from robot_pkg.consts import IDs, STRATEGIES_PATH
from robot_pkg.odometry import OdometryHandler, Odometry
from robot_pkg.step import Move
from robot_pkg.servo import ServoHandler
servo = ServoHandler()

complete = [enum_item.name for enum_item in IDs]
def completer(text, state):
    options = [cmd for cmd in complete if cmd.startswith(text)]
    if state < len(options):
        return options[state]
    else:
        return None

readline.parse_and_bind("tab: complete")
readline.set_completer(completer)

def user_cmd(running:Event):
    while running.is_set():
        try:
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
                    move, success = Move.Distance(p, v, a, attempt=1)
                    if move.wait() and success.is_set():
                        print("Distance finished")

                elif msg_type == IDs.SET_MOTOR_SPEED.name:
                    left = (int)(input("Left motor velocity: "))
                    right = (int)(input("Right motor velocity: "))
                    move, success = Move.Speed(left, right)
                    if move.wait() and success.is_set():
                        print("Speed finished")

                elif msg_type == IDs.SET_MOTOR_RPM.name:
                    left = (int)(input("Left motor RPM: "))
                    right = (int)(input("Right motor RPM: "))
                    move, success = Move.RPM(left, right)
                    if move.wait() and success.is_set():
                        print("RPM finished")
                        
                elif msg_type == IDs.RESET_ODOM.name:
                    x = float(input("New x: "))
                    y = float(input("New y: "))
                    theta = float(input("New theta: "))
                    data = struct.pack('3f', x, y, theta)
                    can_handler.msg_send_queues[IDs[cmd].value].append(data)

                elif msg_type == IDs.SET_SERVO_POSITIONS.name:
    
                    print("Leave a field blank for exit")
                    while True:
                        Id = input("ID: ")
                        if Id == "":
                            break
                    
                        position = input("Position[degree]: ")
                        if position == "":
                            break
                        
                        speed = input("Speed[%]: ")
                        if speed == "":
                            break
                        servo.add_angle(Id, position, speed)

                    servo.sync_write()
                elif msg_type == IDs.GET_SERVO_POSITIONS:
                    Id = input("ID: ")
                    data = struct.pack('I', Id)
                    can_handler.msg_send_queues[IDs[cmd].value].append(data)

                        
                else:
                    print("Message ID is not of sending type")
                    continue                    
            else:
                print("Message ID doesn't exists")
        except KeyboardInterrupt:
            pass

def choose_strategy(color, square, mood):
    temp_strategy = Strategy(color, square, mood)

    for file in os.listdir(STRATEGIES_PATH):
        if file.endswith(".py"):
            strategy = os.path.splitext(file)[0]
            mod = import_module("strategies." + strategy)
            strategy = getattr(mod, strategy)

            if strategy == temp_strategy:
                return strategy
    
    print("STRATEGY NOT FOUND!!!!")
    return None


def main_func():
    main_log = log_handler.get_logger("main")
    main_log.info("Started code")

    args = sys.argv
    strategy =  choose_strategy(args[1], args[2], args[3])
    main_log.info(f"------ Strategy -------\n{strategy}")

    # Open can socket and start sending and receiving threads
    can_handler.start_threads()

    # Start odometry listening thread and initial odometry
    odom = OdometryHandler(Odometry(0.0, 0.0, 90*math.pi/180))
    # odom.start()

    servo.start()

    # Allow user commands
    cmd_debug = Event()
    cmd_debug.set()    # Comment when not testing commands
    cmd_thread = Thread(target=user_cmd, args=(cmd_debug,))
    cmd_thread.start()

    try:
        # pause_queue = can_handler.msg_receive_queues[IDs.GET_PAUSE.value]
        while 1:
        #     # Listen for pause flag on can
        #     if len(pause_queue) > 0:
        #         data = pause_queue.pop()
        #         if data[0]:
        #             paused.set()
        #         else:
        #             paused.clear()

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Cancelling command")
        print("\n")
        pass

    if cmd_thread.is_alive():
        cmd_debug.clear()
        cmd_thread.join()

    odom.stop()
    servo.stop()
    can_handler.stop_threads()


if __name__ == "__main__":
    main_func()
    