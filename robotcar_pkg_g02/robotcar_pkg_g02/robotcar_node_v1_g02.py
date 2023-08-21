# ROS python client
import rclpy
from rclpy.node import Node

# ROS python msg libraries
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# RPi4 libraries and submodulus
import RPi.GPIO as GPIO

# set GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class RobotCarNodeV1G02(Node):
    # Declarando pines como constantes
    PWM_SPEED_PIN = 18  # Cambia este valor al pin deseado
    PWM_STEER_PIN = 12  # Cambia este valor al pin deseado

    def __init__(self):
        super().__init__("RobotCarNodeV1G02") # Redefine node name

        # Usar constantes para los pines
        self.PWM_speed_pin = RobotCarNodeV1G02.PWM_SPEED_PIN
        self.PWM_steer_pin = RobotCarNodeV1G02.PWM_STEER_PIN

        # set GPIO pin mode
        GPIO.setup(self.PWM_speed_pin, GPIO.OUT)
        GPIO.setup(self.PWM_steer_pin, GPIO.OUT)

        # Init PWM obj
        self.PWM_speed = GPIO.PWM(self.PWM_speed_pin, 50.5)  # 50 Hz freq of the speed
        self.PWM_steer = GPIO.PWM(self.PWM_steer_pin, 50.5)  # 50 Hz freq of the steering

        # topic subscriber
        self.cmd_vel_out = self.create_subscription(Twist, '/cmd_vel_out', self.cmd_vel_out_callback, 1)
        self.cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.PWM_speed.start(0.0)
        self.PWM_steer.start(0.0)



    def cmd_vel_callback(self, msg):
        self.speed = msg.linear.x
        self.steer= msg.angular.z
        self.set_motor_speed()
        # if msg.linear.x > 0:
        #     self.PWM_speed.start(8.5)  # 1.7ms pulse
        # elif msg.linear.x < 0:
        #     self.PWM_speed.start(6.5)  # 1.3ms pulse
        # else:
        #     self.PWM_speed.start(7.05)  # 1.5ms pulse, posición neutral
    def cmd_vel_out_callback(self, msg):
        self.speed = msg.linear.x
        self.steer= msg.angular.z
        self.set_motor_speed()
        # if msg.linear.x > 0:
        #     self.PWM_speed.start(8.5)  # 1.7ms pulse
        # elif msg.linear.x < 0:
        #     self.PWM_speed.start(6.5)  # 1.3ms pulse
        # else:
        #     self.PWM_speed.start(7.05)  # 1.5ms pulse, posición neutral

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def set_motor_speed(self):
        msg = "speed: {:.3f}, spin: {:.3f}".format(self.speed,self.steer)
        print(msg)
        speed_map=0
        if self.speed != 0:
            if self.speed < 0:
                speed_map =	self.map((self.speed), -0.5, 0.0, 6.1, 7.05)
                self.PWM_speed.ChangeDutyCycle(speed_map)
            else:
                speed_map =	self.map(abs(self.speed), -0.0, 0.50, 7.05, 8.1)
                self.PWM_speed.ChangeDutyCycle(speed_map)

        else:
            self.PWM_speed.ChangeDutyCycle(7.05)
        steer_map=0
        if self.steer != 0:
            if self.steer < 0:
                steer_map =	self.map((self.steer), -1., 0.0, 6.1, 7.05)
                self.PWM_steer.ChangeDutyCycle(steer_map)
            else:
                steer_map =	self.map(abs(self.steer), -0.0, 1.0, 7.05, 8.1)
                self.PWM_steer.ChangeDutyCycle(steer_map)
        else:
            self.PWM_steer.ChangeDutyCycle(7.05)

        
        msg_clip = "----speed: {:.3f}---- ----steer: {:.3f}----".format(speed_map,steer_map)

        print(msg_clip)


def main(args=None):
    rclpy.init(args=args)  # Inicializar rclpy
    try:
        node = RobotCarNodeV1G02()
        rclpy.spin(node)  # Mantener el nodo en ejecución y escuchar callbacks
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error('Main function encountered error: ' + str(e))
    finally:
        node.destroy_node()  # Destruir el nodo
        rclpy.shutdown()    # Limpiar y cerrar rclpy
        GPIO.cleanup()      # Limpiar los recursos GPIO

if __name__ == "__main__":
    main()