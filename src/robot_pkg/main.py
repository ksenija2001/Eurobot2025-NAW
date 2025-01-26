from logger import LogHandler
from can_controller import CanNetwork, IDs
import time

if __name__ == "__main__":
    log_handler = LogHandler()
    main_log, can_log = log_handler.get_loggers()
    main_log.info("Started code")
    can_handler = CanNetwork(channel='can0', interface='socketcan', max_queue_size=50, log=can_log)
    can_handler.start_threads()

    sent = False
    try:
        start_time = time.time()
        while 1:
            if time.time() - start_time > 5 and not sent:
                sent = True
                can_handler.msg_send_queues[IDs.GET_POSITION.name].put((IDs.GET_POSITION, []))
            
    except KeyboardInterrupt:
        pass

    can_handler.stop_threads()



    