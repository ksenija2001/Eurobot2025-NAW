
from threading import Event, Thread

paused:Event = Event()
import time, os, sys
import struct
import math

from robot_pkg.logger import LogHandler
log_handler = LogHandler()

from robot_pkg.can_controller import CanNetwork
can_handler = CanNetwork(channel='can0', interface='socketcan', max_queue_size=10)
can_handler.init_queues(10)

from robot_pkg.odometry import OdometryHandler, Odometry

from robot_pkg.step import Servo, Move
from robot_pkg.utils import user_cmd #,choose_strategy

def main_func():
    main_log = log_handler.get_logger("main")
    main_log.info("Started code")

    args = sys.argv
    # strategy =  choose_strategy(args[1], args[2], args[3])
    # main_log.info(f"------ Strategy -------\n{strategy}")

    # Open can socket and start sending and receiving threads
    can_handler.start_threads()

    # Start odometry listening thread and initial odometry
    odom = OdometryHandler(Odometry(0.0, 0.0, 90*math.pi/180))
    odom.start()

    Servo.start_threads()

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
    Servo.stop_threads()

    can_handler.stop_threads()


if __name__ == "__main__":
    main_func()
    