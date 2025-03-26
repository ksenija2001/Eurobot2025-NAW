import time, os, sys
from importlib import import_module

from robot_pkg.data import Variables
from robot_pkg.consts import STRATEGIES_PATH
from robot_pkg.execute import Execute
from robot_pkg.old_strategy import Strategy


class Main:

    def __init__(self, color, square, mood):
        strategy = self.choose_strategy(color, square, mood)
        
        self.execute = Execute(strategy)
        self.execute.start()

        self.is_active = False

    def start(self):
        self.is_active = True

    def get_time(self):
        print(f"time = {time.time() - Variables.match_start_time}")

    def stop(self):
        self.execute.stop()
        self.is_active = False

    def choose_strategy(self, color, square, mood):
        temp_strategy = Strategy(color, square, mood)

        for file in os.listdir(STRATEGIES_PATH):
            if file.endswith(".py"):
                strategy = os.path.splitext(file)[0]
                mod = import_module("strategies." + strategy)
                strategy = getattr(mod, strategy)

                if strategy == temp_strategy:
                    return strategy
     
        print("STRATEGY NOT FOUND!!!!")
        exit(1)

if __name__ == "__main__":
    args = sys.argv
    args = ['', 'yellow', 'lower', 'passive']

    if len(args) != 4:
        exit(1)

    main = Main(args[1], args[2], args[3])

    try:
        main.start()
        while main.is_active:
            main.get_time()
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("EXITING")
        main.stop()
        
        