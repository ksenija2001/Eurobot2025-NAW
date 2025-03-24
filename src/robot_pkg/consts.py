import os
from pathlib import Path
from enum import Enum

ROOT_PATH = Path(__file__).parent
LOG_PATH = os.path.join(ROOT_PATH, 'logs')
CONFIG_PATH = os.path.join(ROOT_PATH, 'config')

class IDs(Enum):
    RESET_ODOM  = 0x4F0
    ODOM_CONFIG = 0x4F1

    GET_ODOM = 0x4FF

    SET_MOTOR_SPEED = 0x4D0
    SET_MOTOR_RPM   = 0x4D1
    SET_DISTANCE    = 0x4D2
    SET_XY          = 0x4D3
    SET_ROTATION_FOR = 0x4D4
    SET_ROTATION_TO  = 0x4D5

    GET_MOTOR_SPEED = 0x4DF
    GET_MOVE_DONE   = 0x4DE

    SET_SERVO_POSITIONS = 0x530
    GET_SERVO_POSITIONS = 0x531

    GET_SERVO_IN_POSITION = 0x53F
