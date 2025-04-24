
from threading import Event, Thread

paused:Event = Event()
import time, sys

from robot_pkg.logger import LogHandler
log_handler = LogHandler()

from robot_pkg.can_controller import CanNetwork
can_handler = CanNetwork(channel='can0', interface='socketcan', max_queue_size=10)
can_handler.init_queues(10)

from robot_pkg.lidar import Lidar
from robot_pkg.servo import Servo
from robot_pkg.move import Move
from robot_pkg.in_out import I_O
from robot_pkg.utils import user_cmd,choose_strategy
from robot_pkg.execute import Execute
from robot_pkg.battery import Battery
from robot_pkg.consts import Variables

def main_func():
    main_log = log_handler.get_logger("main")

    battery = Battery()
    battery_state = battery.read_voltage()
    main_log.info(f"Voltage: {battery_state:.2f}")

    # Open can socket and start sending and receiving threads
    can_handler.start_threads()

    Servo.start_threads()
    Move.start_threads()
    I_O.start_threads()
    Lidar.start_threads()

    reset_odom = Move.ResetOdom(125, 1000, 0)#1500 , 125, 1.57)
    reset_odom._execute()

    time.sleep(2)

    running = Event()
    execute = None
    if len(sys.argv) > 1:
        strategy =  choose_strategy(sys.argv[1], sys.argv[2], sys.argv[3])
        if strategy is None:
            raise Exception("To run in debug mode leave the arguments empty, else call 'start_main color square mood'")

        main_log.info(f"------ Strategy -------\n{strategy}")

        Variables.match_start_time = time.time()
        
        running.set()

        execute = Execute(strategy, running)
        execute.start()

    else:
        main_log.info("Debug mode active")
        # Allow user commands
        cmd_debug = Event()
        cmd_debug.set()    # Comment when not testing commands
        cmd_thread = Thread(target=user_cmd, args=(cmd_debug,))
        cmd_thread.daemon = True
        cmd_thread.start()

        running.set()


    try:
        # pause_queue = can_handler.msg_receive_queues[IDs.GET_PAUSE.value]
        while running.is_set():
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

    if execute is not None and execute.thread.is_alive():
        execute.running = False
        execute.thread.join()

    Lidar.stop_threads()
    Servo.stop_threads()
    Move.stop_threads()
    I_O.stop_threads()

    can_handler.stop_threads()


if __name__ == "__main__":
    main_func()
    