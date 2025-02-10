import logging.config
import datetime
import os, shutil

from robot_pkg.paths import CONFIG_PATH, LOG_PATH

LOG_TODAY = os.path.join(LOG_PATH, str(datetime.date.today()))

### tail -f log_filename.log | grep node_name

class LogHandler:

    def __init__(self, old=7) -> None:

        if os.path.exists(LOG_TODAY):
            print("Log directory exists.")
        else:
            os.makedirs(LOG_TODAY)
            print("Log directory created.")
        
        self.remove_logs(old)

        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.log_filename = LOG_TODAY + '/log_' + str(now)

        try:
            logging.config.fileConfig(CONFIG_PATH + '/logging.conf', 
                                    defaults={'log_filename': self.log_filename})
        except:
            print("No logging.conf found")
        
        self.root = logging.getLogger()
        self.root.info("Log started")
        self.root.info(f"Log filename: {self.log_filename}.log")
    
    def get_logger(self, name) -> tuple[logging.Logger]:
        return logging.getLogger(name)

    def remove_logs(self, old) -> None:
        subfolders = [f.name for f in os.scandir(LOG_PATH) if f.is_dir()]
        for f in subfolders:
            day_diff = datetime.date.today() - datetime.datetime.strptime(f, '%Y-%m-%d').date() 
            if abs(day_diff.days) >= old:
                shutil.rmtree(os.path.join(LOG_PATH, f))

    def __del__(self):
        file_size = os.path.getsize(self.log_filename + '.log')
        self.root.info(f"Finished {self.log_filename + '.log'} of size {file_size}")

        if file_size == 0:
            print("Log empty.")
            os.remove(self.log_filename + '.log')

if __name__ == "__main__":
    pass #log = LogHandler()

