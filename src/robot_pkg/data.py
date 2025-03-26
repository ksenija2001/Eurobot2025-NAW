from enum import Enum

class robotData:
    odometryDiameterLeft = 72
    odometryDiameterRight = 72
    odometryDistance = 274.5
    wheelDiameter = 74
    wheelDistance = 150

    motorEncoderPPR = 1024
    wheelReduction = 4/3
    motorReduction = 26

    def QP_to_mm(self, QP) -> float:
        return (self.wheelDiameter * 3.1415926535)/(self.motorEncoderPPR * 4 * self.wheelReduction)
    
    def mm_to_QP(self, mm) ->int:
        return int((self.motorEncoderPPR * 4 * self.motorReduction)/(self.wheelDiameter * 3.1415926535))
    
    def int32_to_bytes(self, num):
        out = []
        out.append((num >> 24) & 0xFF)
        out.append((num >> 16) & 0xFF)
        out.append((num >> 8) & 0xFF)
        out.append((num >> 0) & 0xFF)
        return out

class nucleoData:
    port = '/dev/ttyUSB_NUCLEO'
    baudrate = 115200
    timueOut = 0.1 #sec
    refreshFrequency = 30 #Hz

class Color(Enum):
    BLUE   = 'blue'
    YELLOW = 'yellow'

    def __eq__(self, other:str):
        return self.name.lower() == other

class Square(Enum):
    UPPER  = 'upper'
    CENTER = 'center'
    LOWER  = 'lower'

    def __eq__(self, other:str):
        return self.name.lower() == other


class Mood(Enum):
    PASSIVE   = 'passive'
    AGGRESSIVE = 'aggressive'

    def __eq__(self, other:str):
        return self.name.lower() == other
    
class Variables:
    match_start_time = float('inf')
    points = 0
    

