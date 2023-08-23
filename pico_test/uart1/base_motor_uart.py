
from machine import Pin, I2C, Timer, UART
import machine
import utime

class Base_Motor_Controller():
    uartData = ""
    
    
    def __init__(self,uart_id=0):
        self.uart = UART(uart_id, 9600)  # uart0 uses pins 1 (tx) and 2 (rx)
        
        
    def _uartListen(self):
        while (self.uart.any()):
            uartIn = self.uart.read(1).decode("utf-8")
            if uartIn == "\n":
                self._processData()
                self.uartData = ""
            else:
                self.uartData += uartIn
    
    def _processData(self):
        # print(str(self.uart.any()))
        command = ""
        velocity = 0
        distance = 0
        # print("_processData:" + self.uartData + " " + str(len(self.uartData)))
        if len(self.uartData) == 2:
            command = self.uartData
            if command not in ["SP", "RB", "QT"]:
                command = ""
        if len(self.uartData) == 6:
            command = self.uartData[0:2]
            if command not in ["RV", "TV"]:
                command = ""
            else:
                try:
                    velocity = int(self.uartData[2:6])
                except:
                    command = ""
                    velocity = 0
        if len(self.uartData) == 11:
            command = self.uartData[0:2]
            if command not in ["RD", "TD"]:
                command = ""
            else:
                try:
                    velocity = int(self.uartData[2:6])
                    distance = int(self.uartData[6:11])
                except:
                    command = ""
                    velocity = 0
                    distance = 0
        if len(command) > 0:
            if self.debug:
                print("commandMsg:", self.uartData + " command:" + command +
                      " velocity:" + str(velocity) + " distance:" + str(distance))
            if command == "QT":
                self.stop()
                self.quit()
            elif command == "SP":
                self.stop()
            elif command == "RB":
                self.reboot()
            elif command == "RV":
                self.run(velocity)
            elif command == "TV":
                self.turn(velocity)
            elif command == "RD":
                self.runDistance(velocity, distance)
            elif command == "TD":
                self.turnDistance(velocity, distance)
