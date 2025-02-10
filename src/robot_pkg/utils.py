import subprocess
import argparse
import time

def echo_log():
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', type=str, help='the name of the target')
    parser.add_argument('log', type=str)

    args = parser.parse_args()

    if args.filename == '' or args.log == '':
        print("Specify file and log name")
        exit()

    f = subprocess.Popen(['tail','-F',args.filename, '|' , 'grep', args.log],\
            stdout=subprocess.PIPE,stderr=subprocess.PIPE)

    try:
        while True:
            print(f.stdout.readline())
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass