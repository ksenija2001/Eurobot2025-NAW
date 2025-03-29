
from threading import Event, Thread

paused:Event = Event()
import time, sys, subprocess
import math

from robot_pkg.logger import LogHandler
log_handler = LogHandler()

from robot_pkg.can_controller import CanNetwork
can_handler = CanNetwork(channel='can0', interface='socketcan', max_queue_size=10)
can_handler.init_queues(10)

from robot_pkg.odometry import OdometryHandler, Odometry

from robot_pkg.step import Servo, Move
from robot_pkg.utils import user_cmd,choose_strategy
from robot_pkg.execute import Execute

def main_func():
    main_log = log_handler.get_logger("main")

    # Open can socket and start sending and receiving threads
    can_handler.start_threads()

    # Start odometry listening thread and initial odometry
    odom = OdometryHandler(Odometry(0.0, 0.0, 90*math.pi/180))
    odom.start()

    Servo.start_threads()

    if len(sys.argv) > 1:
        strategy =  choose_strategy(sys.argv[1], sys.argv[2], sys.argv[3])
        if strategy is None:
            raise Exception("To run in debug mode leave the arguments empty, else call 'start_main color square mood'")

        main_log.info(f"------ Strategy -------\n{strategy}")

        execute = Execute(strategy)
        execute.start()
    else:
        main_log.info("Debug mode active")

        # Allow user commands
        cmd_debug = Event()
        cmd_debug.set()    # Comment when not testing commands
        cmd_thread = Thread(target=user_cmd, args=(cmd_debug,))
        cmd_thread.daemon = True
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
        print("Cancelling")
        print("\n")

    if execute.thread.is_alive():
        execute.is_active = False
        execute.thread.join()

    odom.stop()
    Servo.stop_threads()

    can_handler.stop_threads()


if __name__ == "__main__":
    main_func()
    