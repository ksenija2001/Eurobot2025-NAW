from smbus import SMBus
import time

I2C_ADRESS = 0x4d
I2C_DEVICE = 1
VOLTAGE_MAX = 16.8
VOLTAGE_MIN = 13.2

class Battery:

    def __init__(self):
        self.bus = SMBus(I2C_DEVICE)

    def read_voltage(self):
        '''
            Samples voltage 100 times in the span of 1s.
        '''
        readings = []
        for i in range(0, 100):
            readings.append(self.get_reading())
            time.sleep(0.01) # 10ms
        
        battery_state = sum(readings)/100
        return battery_state
    
    def get_reading(self):
        '''
            Reads two bytes of data from ADC device that correspond to the latest voltage reading.
        '''
        self.bus.read_byte(I2C_ADRESS)
        data = self.bus.read_i2c_block_data(I2C_ADRESS, 0, 2)

        data = (data[0] << 8) | data[1]
  
        conv_data = self.convert(data)
        return conv_data

    def convert(self, adc_value):
        '''
            Returns percentage from voltage value.
        '''
        voltage = 3.3/4095.0 * adc_value * (4.7+1)/1.0
        percentage = (voltage - VOLTAGE_MIN) * 100 / (VOLTAGE_MAX - VOLTAGE_MIN) 
        if percentage < 0:
            return 0
        elif percentage > 100:
            return 100
        
        return percentage
