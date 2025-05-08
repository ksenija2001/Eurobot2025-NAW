import os
from pathlib import Path
from enum import Enum
from threading import Event
# from robot_pkg.strategy import Color

ROOT_PATH = Path(__file__).parent
LOG_PATH = os.path.join(ROOT_PATH, 'logs')
CONFIG_PATH = os.path.join(ROOT_PATH, 'config')
STRATEGIES_PATH = os.path.join(ROOT_PATH, 'strategies')

IP = "192.168.50.219"

class Variables:
    match_start_time = float('inf')
    front_detection = Event()
    back_detection = Event()
    processing_detection = Event()
    points = 0
    color = 'blue'

class LIDAR_FOV:
    FRONT_DEPLOY = 500 
    FRONT_UNDEPLOY = 500
    BACK_DEPLOY = 600
    BACK_UNDEPLOY = 450

class ODOM_DATA:
    diameter = 73 #73.01562845724801
    # odom_right_diameter = 73 #72.98437197283523
    left_gain = 1.00023541410066
    right_gain = 0.9997645755745481
    inc_mm = 0.028050989999999998 #0.027995 #0.028018311194310475
    odom_track = 347.5596728779546 #329.2836088607495

class Points:
    LEVEL1 = 4
    LEVEL2 = 8
    LEVEL3 = 16
    BANNER = 20
    HOME = 10

class IDs(Enum):
    SET_LIDAR = 0x4C0
    SET_LIDAR_ODOM = 0x4C1
    
    GET_BEACON     = 0x4CD
    GET_OPPONENT   = 0x6CE
    GET_DETECTION  = 0x4CF
    
    RESET_ODOM  = 0x4F0
    ODOM_CONFIG = 0x4F1

    SET_MOTOR_SPEED  = 0x4D0
    SET_MOTOR_RPM    = 0x4D1
    SET_DISTANCE     = 0x4D2
    SET_ROTATION_FOR = 0x4D3
    SET_ROTATION_TO  = 0x4D4
    SET_XY           = 0x4D5
    SET_SPLINE       = 0x4D6
    SET_STOP         = 0x4D7
    SET_DETECTION    = 0x4D8

    GET_MOTOR_SPEED = 0x4DF
    GET_MOVE_DONE   = 0x4AE

    SET_SERVO_POSITIONS = 0x530
    GET_SERVO_POSITIONS = 0x531
    SET_RC_SERVO_POSITIONS = 0x532
    SET_SERVO_TORQUE = 0x533

    GET_SERVO_IN_POSITION = 0x53F
    GET_SERVO_ERROR = 0x53E

    SET_IO = 0x690
    GET_IO = 0x69F

    GET_ODOM = 0x6FF

    UNKNOWN = 0x88


    @classmethod
    def has_key(cls, name):
        return name in cls.__members__
