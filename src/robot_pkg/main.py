
from robot_pkg.logger import LogHandler
from robot_pkg.can_controller import CanNetwork, IDs
from robot_pkg.odometry import OdometryHandler, Odometry
import time
import can
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

def main_func():
    log_handler = LogHandler()
    main_log = log_handler.get_logger("main")
    can_log = log_handler.get_logger("can")
    odom_log = log_handler.get_logger("odom")

    main_log.info("Started code")
    can_handler = CanNetwork(channel='can0', interface='socketcan', max_queue_size=10, log=can_log)
    can_handler.start_threads()

    odom = OdometryHandler(can_handler.msg_receive_queues[IDs.GET_ODOM.value],
                           can_handler.msg_send_queues[IDs.RESET_ODOM.value],
                           odom_log,
                           Odometry(0.0, 0.0, 90*math.pi/180))

    odom.start()

    time.sleep(1)
    try:
        while 1:
            cmd = input("Message ID: ")

            try:
                if IDs.has_key(cmd):
                    output = []
                    frmt = ''
                    msg_type = IDs[cmd].name

                    if msg_type == IDs.SET_DISTANCE.name:
                        p = input("Target position: ")
                        v = input("Target velocity: ")
                        a = input("Target acceleratioon: ")
                        frmt = 'f'
                        output = [float(p), float(v), float(a)]
                    elif msg_type == IDs.SET_MOTOR_SPEED.name:
                        left = (int)(input("Left motor velocity: "))
                        right = (int)(input("Right motor velocity: "))
                        frmt = 'i'
                        output = [left, right]
                    elif msg_type == IDs.SET_MOTOR_RPM.name:
                        left = (int)(input("Left motor RPM: "))
                        right = (int)(input("Right motor RPM: "))
                        frmt = 'i'
                        output = [left, right]
                    elif msg_type == IDs.RESET_ODOM.name:
                        x = input("New x: ")
                        y = input("New y: ")
                        theta = input("New theta: ")
                        frmt = 'f'
                        output = [float(x), float(y), float(theta)]
                    else:
                        print("Message ID is not of sending type")
                        continue

                    print(f"output: {output}, length: {len(output)}")
                    data = struct.pack(frmt*len(output), *output)
                    can_handler.msg_send_queues[IDs[cmd].value].append(data)
                else:
                    print("Message ID doesn't exists")
            except KeyboardInterrupt:
                pass

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Cancelling command")
        print("\n")
        pass

    odom.stop()
    can_handler.stop_threads()


if __name__ == "__main__":
    main_func()
    