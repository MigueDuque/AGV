# receiver.py / Tx/Rx => Tx/Rx
import os
from machine import UART, Timer
import machine
from time import sleep

class Base_Motor_Controller():
        
    def __init__(self,uart_id=0):
        self.uart = UART(uart_id, 115200)  # uart0 uses pins 1 (tx) and 2 (rx)
        print(self.uart)
     
     
     
    def _monitorStatus(self, timer):          
        self._uartListen()
        print('start')
           
    def start(self, debug=False):
        
        self.debug = debug
        self.statusTimer = Timer()
        self.statusTimer.init(freq=10, mode=Timer.PERIODIC,
                              callback=self._monitorStatus)
        
   
         
    def _uartListen(self):
        while (self.uart.any()):
            uartIn = self.uart.read(1).decode("utf-8")
            if uartIn == "\n":
                self._processData()
                self.uartData = ""
            else:
                self.uartData += uartIn

    def _processData(self):
        
        command = ""
        velocity = 0
        distance = 0
        print("_processData:" + self.uartData + " " + str(len(self.uartData)))


''' 
uart = machine.UART(0, 115200)
print(uart)
b = None
msg = ""
'''

while True:
    
    motor = Base_Motor_Controller()
    motor.start(True)
    
    sleep(0.1)
    ''' 
    sleep(0.1)
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
    '''