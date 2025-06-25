import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from time import sleep

MOTOR_PWM_PIN = 13  # GPIO13 (ピン番号33)

class MotorPWM(Node):
    def __init__(self):
        super().__init__('motor_pwm_toggle')

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_PWM_PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(MOTOR_PWM_PIN, 1000)  # 1kHz
        self.pwm.start(0)  # 初期は停止

        self.speed_high = 100  # 最大出力（100%）
        self.speed_low = 0     # 最小出力（0%）
        self.toggle = True

        # 5秒ごとに速度切り替え
        self.timer = self.create_timer(5.0, self.toggle_speed)

        self.get_logger().info("5秒ごとにモーター速度を切り替えます")

    def toggle_speed(self):
        if self.toggle:
            self.pwm.ChangeDutyCycle(self.speed_high)
            self.get_logger().info(f"🟢 高速 {self.speed_high}% 出力中")
        else:
            self.pwm.ChangeDutyCycle(self.speed_low)
            self.get_logger().info(f"⚪ 停止 {self.speed_low}% 出力中")
        self.toggle = not self.toggle

    def destroy_node(self):
        self.get_logger().info("ノード終了、GPIOクリーンアップ")
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorPWM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
