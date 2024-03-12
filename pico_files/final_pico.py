from machine import I2C, Pin, UART
import utime
import ustruct
import sys
import time
############################
pca9685_addr = 0x40
channel0 = 0x06
i2c = I2C(0,
            scl=Pin(17),
            sda=Pin(16),
            freq=400000)

uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
channels = 6 
rxValues = [0]*channels
loop_time = 0.1
##########3333333333333333
def process_stm_data(data, channels):
    """
    Each Channel give output in range (1000, 2000).
    2 bytes are recieved for each channel

    """
    list = [0]*channels
    
    list[0] = (data[2] | data[3]<<8);
    list[1] = (data[4] | data[5]<<8);
    list[2] = (data[6] | data[7]<<8);
    list[3] = (data[8] | data[9]<<8);
    list[4] = (data[10] | data[11]<<8);
    list[5] = (data[12] | data[13]<<8);
    
    return list

class robotic_arm:
    def __init__(self, i2c = i2c, address = pca9685_addr, channel0 = channel0):
        self.i2c = i2c
        self.address = address
        self.channel0 = channel0
        self.write(0x00, 0x00)
        self.counts_in_one_cycle = 4096
        self.min_duty = 0.025
        self.max_duty = 0.125
        
    def write(self, register, data):
        msg = bytearray()
        msg.append(data)
        self.i2c.writeto_mem(self.address, register, msg)
        
    def read(self, register):
        msg = self.i2c.readfrom_mem(self.address, register, 1)[0]
        return msg
    
    def set_freq(self, freq):

        prescale = int(25000000.0 / 4096.0 / freq)
        old_mode = self.read(0x00) # Mode 1
    
        # print((old_mode & 0x7F) | 0x10)
        self.write(0x00, (old_mode & 0x7F) | 0x10) #sleep
        self.write(0xfe, prescale) # Prescale
        self.write(0x00, old_mode) # Mode 1
        time.sleep(0.001)
        self.write(0x00, old_mode | 0xa1)
        
    def pwm_gen(self, on, off, index):
    
        data = ustruct.pack('<HH', on, off)
        #print(data)
        self.i2c.writeto_mem(self.address, self.channel0 + 4 * index,  data)
    def set_joint_angle(self, joint_index, angle):
        #angle in degrees
        on = int(0)
        off = int(self.counts_in_one_cycle * (self.min_duty + (self.max_duty - self.min_duty) * angle / 180))
        #print(on, off, joint_index)
        self.pwm_gen(on = on, off = off, index = joint_index)
    def get_angles(self,rxValues):
        
        



'''
for index in [0,1,13,14,15]:
        robotic_arm.set_joint_angle(index, 90)
'''
if __name__ == '__main__':
    
    robotic_arm = robotic_arm()
    robotic_arm.set_freq(50)
    t_start = time.time()
    angle = 0
    store_delta = 0
    delta_angle = store_delta
    
    while(1):
        #########
        
        if time.time() - t_start > 60:  
            print('Finished')
            break
        
        #########
        
        received_data = uart.read(32)
        if isinstance(received_data, bytes) != 1 or len(received_data) < channels :
            continue
        received_data = list(received_data)
        rxValues = process_stm_data(received_data, channels)
        print("Received data:", rxValues)
        angle_values = robotic_arm.get_angles(rxValues)
        '''
        angle_values = []
        for value in rxValues:
            angle_values.append(180*(value - 1000)/1000)
        '''
        ##########
   
        for i in range(4):
            robotic_arm.set_joint_angle(i*4, angle_values[i])
   
        
            
        
        
        time.sleep(loop_time)
