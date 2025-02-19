
from robot_pkg.logger import LogHandler
from robot_pkg.can_controller import CanNetwork, IDs
from robot_pkg.odometry import OdometryHandler, Odometry
from robot_pkg.step import Move
from threading import Event, Thread
import time
import struct
import math
import readline

complete = [enum_item.name for enum_item in IDs]

def completer(text, state):
    options = [cmd for cmd in complete if cmd.startswith(text)]
    if state < len(options):
        return options[state]
    else:
        return None

readline.parse_and_bind("tab: complete")
readline.set_completer(completer)

# Global access variables
can_handler:CanNetwork
log_handler:LogHandler
paused:Event

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
                
                else:
                    print("Message ID is not of sending type")
                    continue                    
            else:
                print("Message ID doesn't exists")
        except KeyboardInterrupt:
            pass


def main_func():
    log_handler = LogHandler()
    main_log = log_handler.get_logger("main")
    main_log.info("Started code")

    # Open can socket and start sending and receiving threads
    can_handler = CanNetwork(channel='can0', interface='socketcan', max_queue_size=10)
    can_handler.start_threads()

    # Start odometry listening thread and initial odometry
    odom = OdometryHandler(Odometry(0.0, 0.0, 90*math.pi/180))
    odom.start()

    # Allow user commands
    cmd_debug = Event()
    cmd_debug.set()    # Comment when not testing commands
    cmd_thread = Thread(target=user_cmd, args=(cmd_debug,))
    cmd_thread.start()

    try:
        pause_queue = can_handler.msg_receive_queues[IDs.GET_PAUSE.value]
        while 1:
            # Listen for pause flag on can
            if len(pause_queue) > 0:
                data = pause_queue.pop()
                if data[0]:
                    paused.set()
                else:
                    paused.clear()

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Cancelling command")
        print("\n")
        pass

    if cmd_thread.is_alive():
        cmd_debug.clear()
        cmd_thread.join()

    odom.stop()
    can_handler.stop_threads()

if __name__ == "__main__":
    main_func()
    