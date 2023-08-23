import machine
import utime

led = machine.Pin(25, machine.Pin.OUT)

while True:
    led.toggle()
    print("on\n")
    utime.sleep_ms(100)