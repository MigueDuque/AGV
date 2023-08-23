# libreria machine interaccion hardware
import machine
# libreria para el manejo de tiempo
import utime

# funcion que llama el timer
def ticks(timer):
    global led
    led.toggle()
    print("timer loop")

# instacia objeto led, Clase Pin --> keyword: Pin.OUT
led = machine.Pin(25, machine.Pin.OUT)

#timer = machine.Timer()
#timer.init(freq = 2.5, mode = machine.Timer.PERIODIC, callback=ticks)

while True:
    print("main loop")
    utime.sleep_ms(100)