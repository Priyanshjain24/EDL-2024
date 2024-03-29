from machine import I2C, Pin, UART
import utime
import ustruct
import sys
import time
from math import atan2, acos, sin, cos, asin, sqrt
from math import pi as PI

RADIAN_TO_DEGREE = 180 / PI
DEGREE_TO_RADIAN = PI / 180
DISPLACEMENT_SCALE = 3
BASE_SPEED_SCALE_R = 1
BASE_SPEED_SCALE_L = 20

l1 = 10.5
l2 = 14.7
t1_offset = 30
t2_offset = 30

t1_l = -30 # To check after doing IK, before applying offset
t1_h = 150
t2_l = -60
t2_h = 120

HOME_THETA1 = 0 # degrees
HOME_THETA2 = 0 # degrees
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
loop_time = 0.02
##########3333333333333333
def clip(a, m, M):
    return min(max(a, m), M)

def process_stm_data(data, channels):
    """
    Each Channel give output in range (1000, 2000).
    2 bytes are recieved for each channel
    """
    list = [0]*channels
    
    list[0] = (data[2] | data[3]<<8)
    list[1] = (data[4] | data[5]<<8)
    list[2] = (data[6] | data[7]<<8)
    list[3] = (data[8] | data[9]<<8)
    list[4] = (data[10] | data[11]<<8)
    list[5] = (data[12] | data[13]<<8)

    for i in range(4):
        list[i] = (list[i] - 1500) / 500
    list[4] = (list[4] - 1000) / 500
    list[5] = (list[5] - 1000) / 500
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
        self.x, self.y = self.getXY(HOME_THETA1, HOME_THETA2) 

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

    def getXY(self, t1, t2):
        t1 = t1 * DEGREE_TO_RADIAN
        t2 = t2 * DEGREE_TO_RADIAN
        x = l1 * sin(t1) + l2 * sin(t1 + t2)
        y = l1 * cos(t1) + l2 * cos(t1 + t2)
        return x, y
    
    def getThetas(self, x, y):
        '''
        Solves IK for x and y 
        '''
        d2 = x**2 + y**2
        phi = atan2(y, x)
        a = (d2 - l1**2 - l2**2) / (2 * l1 * l2)
        a = clip(a, -1, 1)
        theta2 = acos(a)
        alpha = asin(sin(theta2) * l2 / sqrt(d2))
        theta1 = PI/2 - phi - alpha
        theta1 = theta1 * RADIAN_TO_DEGREE
        theta2 = theta2 * RADIAN_TO_DEGREE
        return theta1, theta2

    def get_angles(self,rxValues):
        received_data = []
        for i in rxValues :
            received_data.append(180 * (i - 1000)/1000)    
        
        return received_data

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
    angle_values = [90]*6
    X,Y = [14.5, 10]
    while(1):
        #########
        if time.time() - t_start > 300:  
            print('Finished')
            break
        
        #########
        
        received_data = uart.read(32)
        if isinstance(received_data, bytes) != 1 or len(received_data) < channels :
            continue
        received_data = list(received_data)
        rxValues = process_stm_data(received_data, channels)
        #print("Received data:", rxValues)
        #angle_values = robotic_arm.get_angles(rxValues)
        '''
        angle_values = []
        for value in rxValues:
            angle_values.append(180*(value - 1000)/1000)
        '''
        ##########
        #mode = int(angle_values[5]/90)
        #print(angle_values)
        #print(mode)
        mode = rxValues[5]
        
               
        
        for i in range(len(angle_values)):
            angle_values[i] = clip(angle_values[i], 0, 180)
        
        
        if mode == 1 :
            
            angle_values[0] += rxValues[0] * DISPLACEMENT_SCALE
            angle_values[1] += rxValues[1] * DISPLACEMENT_SCALE
            #rxValues[2] = -rxValues[2]

            angle_values[2] += rxValues[2] * DISPLACEMENT_SCALE
            rxValues[3] = -rxValues[3]
            if rxValues[3] < 0 :
                angle_values[3] = 90 + rxValues[3] * BASE_SPEED_SCALE_R
            else :
                angle_values[3] = 90 + rxValues[3] * BASE_SPEED_SCALE_L
                
   #         robotic_arm.set_joint_angle(1, angle_values[3])
      #      robotic_arm.set_joint_angle(3, angle_values[2])
     #       robotic_arm.set_joint_angle(4, angle_values[1])
       #     robotic_arm.set_joint_angle(7, angle_values[0])
            
        elif mode == 2 :
            angle_values[5] += rxValues[3] * DISPLACEMENT_SCALE
            angle_values[5] = clip(angle_values[5], 40, 95)
            angle_values[4] += rxValues[2] * DISPLACEMENT_SCALE
            angle_values[4] = clip(angle_values[4], 0, 180)
            
            #theta1 = angle_values[2] - 39.34
            #theta2 = 142 - angle_values[1]
            #delTheta1 = rxValues[0] * DISPLACEMENT_SCALE
            #tdelt2 = -l1 * sin(theta1) / (l2 * sin(theta1 + theta2)) -1
            #delTheta2 = tdelt2 * delTheta1
            #theta1 += delTheta1
            #theta2 += delTheta2
            X = X + rxValues[0] * 0.2
            Y = Y + rxValues[1] * 0.2
            theta1, theta2 = robotic_arm.getThetas(X, Y)
            
            angle_values[2] = theta1 + 39.34
            angle_values[1] = 142 - theta2
            
            angle_values[2] = clip(angle_values[2], 0, 180)
            angle_values[1] = clip(angle_values[1], 0, 180)
            print(robotic_arm.getXY(angle_values[2] - 39.34, 142 - angle_values[1]))
            #print(angle_values[2] - 39.34, 142 - angle_values[1])
            
        robotic_arm.set_joint_angle(3, angle_values[2])
        robotic_arm.set_joint_angle(4, angle_values[1])
        robotic_arm.set_joint_angle(8, angle_values[4])
        robotic_arm.set_joint_angle(11, angle_values[5])
        robotic_arm.set_joint_angle(7, angle_values[0])
        X, Y = robotic_arm.getXY(angle_values[2] - 39.34, 142 - angle_values[1])
        #for i in range(4):
    
            #robotic_arm.set_joint_angle(i*4, angle_values[i])
        #print(angle_values)
        
        time.sleep(loop_time)


