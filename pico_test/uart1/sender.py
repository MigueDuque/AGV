# sender.py
import time
import serial
ser = serial.Serial(
  port='/dev/ttyS0', # Change this according to connection methods, e.g. /dev/ttyUSB0
  baudrate = 115200,
  parity=serial.PARITY_NONE,
  stopbits=serial.STOPBITS_ONE,
  bytesize=serial.EIGHTBITS,
  timeout=1
)
msg = ""
i = 0
j = 0
while True:
    i+=1
    j-=1
    msg = "r{},l{}".format(i,j)
    print(msg)
    ser.write(msg.encode('utf-8'))
    
    
    msg_read=ser.readline()
    print(msg_read)
    time.sleep(0.1)