import tm1637 
from threading import Thread, Lock
import time

CLK_GPIO = 26
DIO_GPIO = 16

class Display:
    def __init__(self):
        self.display = tm1637.TM1637(CLK_GPIO, DIO_GPIO)
        self.points = 0
        self.lock = Lock()
        self.running = False
        self._thread = Thread(target=self.loop)

    def add_points(self, points):
        self.lock.acquire()
        self.points += points
        self.lock.release()

    def start(self):
        self.running = True
        self._thread.start()

    def stop(self):
        self.running = False
        self._thread.join()
        # print("Display stopped")

    def loop(self):
        while self.running:
            self.lock.acquire()
            p = self.points
            self.lock.release()
            
            self.display.number(p)

            time.sleep(0.1)

if __name__ == "__main__":
    display = Display()
    try:
        while True:
            num = input("Unesi broj: ")
            display.setNumber(int(num))

    except KeyboardInterrupt:
        pass
