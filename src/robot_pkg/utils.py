import time, datetime
import os, glob, subprocess, argparse

from robot_pkg.consts import LOG_PATH

def echo_log():
    parser = argparse.ArgumentParser()
    LOG_TODAY = os.path.join(LOG_PATH, str(datetime.date.today()))
    latest_log = max(glob.glob(LOG_TODAY + "/*"), key=os.path.getctime)

    #parser.add_argument('filename', type=str, help='the name of the target')

    # parser.add_argument('log', type=str)

    # args = parser.parse_args()

    # if args.log == '':
    #     print("Specify file and log name")
    #     exit()

    # print(args.log)
    path = os.path.join(LOG_TODAY, latest_log)
    f = subprocess.Popen(['tail','-F', path],
            stdout=subprocess.PIPE,stderr=subprocess.PIPE)

    try:
        while True:
            print(f.stdout.readline())
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass