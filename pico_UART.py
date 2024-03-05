from machine import UART, Pin
import time

# Configure UART pin 0 - tx, pin 1 - rx
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))


channels = 6 # 6 Channels
rxValues = [0]*channels
"""
rxValues:

0 : Right Horizontal
1 : Right Vertical
2 : Left Vertical
3 : Left Horizontal
4 : Switch A
5 : Switch B
"""

def process_data(data, channels):
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
    

loop_time = 0.1
while True:
    
    # Receive 32 bytes from STM32
    received_data = uart.read(32)
    
    # Ignore if data is not valid
    if isinstance(received_data, bytes) != 1 or len(received_data) < channels :
        continue
    
    # Convert recived data to list
    received_data = list(received_data)
    
    rxValues = process_data(received_data, channels)

    print("Received data:", rxValues)
    
    time.sleep(loop_time)
