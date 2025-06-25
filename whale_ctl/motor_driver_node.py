import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

MOTOR_PWM_PIN = 13  # GPIO13 (ピン番号: 13)

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # GPIOセットアップ
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_PWM_PIN, GPIO.OUT)

        self.pwm = GPIO.PWM(MOTOR_PWM_PIN, 1000)  # 1kHzでPWM生成
        self.pwm.start(0)  # 最初は停止

        # /joyサブスクライブ
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info('モーター制御ノードが起動しました（ボタン1でON）')

    def joy_callback(self, msg):
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:
            self.get_logger().info('🟢 ボタン1が押された：モーター ON')
            self.pwm.ChangeDutyCycle(70)  # 70%出力でON（調整可）
        else:
            self.pwm.ChangeDutyCycle(0)
            self.get_logger().info('⚪ ボタン1が離された：モーター OFF')

    def destroy_node(self):
        self.get_logger().info('ノード終了、GPIOをクリーンアップします')
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
