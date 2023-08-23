# receiver.py / Tx/Rx => Tx/Rx
import os
from machine import UART, Timer
import machine
import utime

  
def _uartListen():
    global uart, uartData
    while (uart.any()):
        uartIn = uart.readline().decode("utf-8")
        if uartIn == "\n":
            _processData()
            uartData = ""
        else:
            uartData += uartIn
 
def _processData(self):
    global uart, uartData
    command = ""
    velocity = 0
    distance = 0
    print("_processData:" + uartData + " " + str(len(uartData)))

uart_id = 0
uart = UART(uart_id, 115200)  # uart0 uses pins 1 (tx) and 2 (rx)
print(uart)   

# time setup
previous_time = 0
sample_time = 400

b = None
msg = ""
while True:
    # time management
    current_time = utime.ticks_ms()
    print(current_time)
    delta_time = utime.ticks_diff(current_time, previous_time)
    
    # sampling conditional
    if delta_time > sample_time:
        if uart.any():
        
            b = uart.readline()
            #print(type(b))
            #print(b)
            try:
                msg = b.decode('utf-8')
                #print(type(msg))
                print(">> " + msg)
            except:
                pass
 
        previous_time = current_time        
    ''' 
    sleep(0.1)

    '''