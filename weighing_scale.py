import serial
import re
import keyboard
import time

class WeighingScale():
    def __init__(self, port='/dev/ttyUSB0'):
        self.serial_port = serial.Serial(port, 9600, timeout=1)

    def read_weight(self):
        '''
        Return weight value in grams
        '''
        # clear buffer
        self.serial_port.reset_input_buffer()
        serial_data = self.serial_port.readline().decode('utf-8').strip() # readline blocks by default
        if (serial_data != ''):
            try:
                weight = float(re.findall(r'-?\d+\.?\d*', serial_data)[0])
                return weight
            except ValueError:
                print(f"Invalid data received: {serial_data}")
        else:
            print("No data recieved")
            return -1
    
    def weight_averaged(self, N = 5):
        '''
        Return average of N weight scale reads
        '''
        weights = []
        for i in range(N):
            weight = self.read_weight()
            if weight != -1:
                weights.append(self.read_weight())
        return sum(weights) / len(weights)
    
    def read_weight_on_key(self):
        print("Press 'w' to get the weight...")
        while True:
            if keyboard.is_pressed('w'): 
                weight = self.read_weight()
                if weight != -1:
                    print(f"\nWeight: {weight} grams")
                    return weight

if __name__ == '__main__':
    scale = WeighingScale()
    scale.read_weight_on_key()