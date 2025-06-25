import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

MOTOR_PWM_PIN = 13  # GPIO13 (ピン33)

class MotorButtonControl(Node):
    def __init__(self):
        super().__init__('motor_pwm_button_control')

        # GPIO初期化
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_PWM_PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(MOTOR_PWM_PIN, 1000)  # 1kHz
        self.pwm.start(0)

        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info("ジョイスティックのボタンでモーター出力を制御します")

    def joy_callback(self, msg):
        speed = 0

        # 安全確認（長さ不足でエラー回避）
        buttons = msg.buttons if len(msg.buttons) >= 2 else [0, 0]

        if buttons[0] == 1:  # ボタン1（index 0）
            speed = 100
        elif buttons[1] == 1:  # ボタン2（index 1）
            speed = 50
        else:
            speed = 0

        self.pwm.ChangeDutyCycle(speed)
        self.get_logger().info(f"ボタン入力 → PWM出力: {speed}%")

    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("ノード終了、GPIOクリーンアップ")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorButtonControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
