import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

from whale_ctl.mecanum_ctl import generate_motor_cmd

class ActuatorDriver(Node):
    def __init__(self):
        super().__init__('actuator_driver')

        # pins setup for wheel
        self.motor_pins = {
            0: {'dir': 8,  'pwm': 27},
            1: {'dir': 18, 'pwm': 12},
            2: {'dir': 25, 'pwm': 16},
            3: {'dir': 26, 'pwm': 20},
            4: {'dir': 6,  'pwm': 5},  # chain
        }

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        self.pwms = {}
        for mid, pins in self.motor_pins.items():
            GPIO.setup(pins['dir'], GPIO.OUT)
            GPIO.setup(pins['pwm'], GPIO.OUT)
            pwm = GPIO.PWM(pins['pwm'], 1000)  # 1kHz PWM
            pwm.start(0)  # set freq 0%
            self.pwms[mid] = pwm
            self.get_logger().info(
                f"Motor {mid} initialized (DIR={pins['dir']}, PWM={pins['pwm']})"
            )

        # /joy
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info('Subscribed to /joy — actuator driver ready')

        # chain control
        self.power = 0.2

    def joy_callback(self, msg: Joy):
        # axis[1] : v_x, axis[0] : v_y, axis[3] : v_yaw
        v_x   = msg.axes[1] * 100
        v_y   = msg.axes[0] * 100
        v_yaw = msg.axes[2] * 100

        wheel_speeds = generate_motor_cmd(v_x, v_y, v_yaw)
        self.get_logger().info(f"cmd: {wheel_speeds[0]}, {wheel_speeds[1]}, {wheel_speeds[2]}, {wheel_speeds[3]}")

        i=0
        for vel in wheel_speeds:
            self.move_motor(i, vel)
            i+=1

        # button up
        if msg.buttons[7] == 1:
            self.move_motor(4, 100*self.power)
            self.get_logger().info("chain forward")
        # button down
        elif msg.buttons[5] == 1:
            self.move_motor(4, -100*self.power)
            self.get_logger().info("chain backward")
        else:
            self.move_motor(4, 0)
            self.get_logger().info("chain stop")
            

    def move_motor(self, motor_id: int, vel: float):
        if motor_id not in self.motor_pins:
            self.get_logger().warn(f"Invalid motor ID: {motor_id}")
            return

        pins = self.motor_pins[motor_id]
        duty = max(0, min(abs(vel), 100))

        GPIO.output(pins['dir'], GPIO.HIGH if vel < 0 else GPIO.LOW)
        self.pwms[motor_id].ChangeDutyCycle(duty)


    def destroy_node(self):
        self.get_logger().info("Shutting down — stopping all motors & cleaning up GPIO")
        for pwm in self.pwms.values():
            pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

