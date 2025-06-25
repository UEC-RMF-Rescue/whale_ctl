import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO

MOTOR_PWM_PIN = 13  # BCM 13 (ピン33)

class MotorPWM(Node):
    def __init__(self):
        super().__init__('motor_pwm_gradient')

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_PWM_PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(MOTOR_PWM_PIN, 1000)
        self.pwm.start(0)

        self.duty = 0
        self.direction = 1  # 1: 上昇, -1: 下降
        self.step = 5       # 一度に増減させる%（グラデーションのなめらかさ）
        self.max_duty = 100
        self.min_duty = 0

        self.timer = self.create_timer(0.1, self.update_pwm)  # 0.1秒ごとに変化
        self.get_logger().info("PWMグラデーション制御を開始（0.1秒ごと）")

    def update_pwm(self):
        self.pwm.ChangeDutyCycle(self.duty)
        self.get_logger().info(f"PWM出力: {self.duty}%")

        self.duty += self.step * self.direction

        # 上限・下限で方向反転
        if self.duty >= self.max_duty:
            self.duty = self.max_duty
            self.direction = -1
        elif self.duty <= self.min_duty:
            self.duty = self.min_duty
            self.direction = 1

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("PWM制御終了、GPIOクリーンアップ")
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
