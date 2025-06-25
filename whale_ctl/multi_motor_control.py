import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

class MultiMotorControl(Node):
    def __init__(self):
        super().__init__('multi_motor_control')

        # モーターIDごとの DIR / PWM ピン
        self.motor_pins = {
            0: {'dir': 8,  'pwm': 27},
            1: {'dir': 24, 'pwm': 13},
            2: {'dir': 25, 'pwm': 16},
            3: {'dir': 26, 'pwm': 20},
        }

        # GPIO初期化
        GPIO.setmode(GPIO.BCM)
        self.pwms = {}

        for motor_id, pins in self.motor_pins.items():
            GPIO.setup(pins['dir'], GPIO.OUT)
            GPIO.setup(pins['pwm'], GPIO.OUT)
            pwm = GPIO.PWM(pins['pwm'], 1000)  # 1kHz
            pwm.start(0)
            self.pwms[motor_id] = pwm
            self.get_logger().info(
                f"モーター{motor_id} 初期化完了 (DIR: {pins['dir']}, PWM: {pins['pwm']})"
            )

        # /joy トピック購読
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.get_logger().info('🎮 /joy 購読開始: ボタン1/4で全モーター制御')

    def move_motor(self, motor_id: int, vel: float):
        """指定IDのモーターを vel(−100〜100) で動かす"""
        if motor_id not in self.motor_pins:
            self.get_logger().warn(f"⚠️ 無効なモーターID: {motor_id}")
            return

        pins = self.motor_pins[motor_id]
        duty = max(0, min(abs(vel), 100))
        # MD10C R3 は HIGH=後退, LOW=前進
        GPIO.output(pins['dir'], GPIO.HIGH if vel < 0 else GPIO.LOW)
        self.pwms[motor_id].ChangeDutyCycle(duty)
        self.get_logger().info(
            f"モーター{motor_id} → 速度: {vel}% （DIR: {'後退' if vel < 0 else '前進'}）"
        )

    def joy_callback(self, msg: Joy):
        # ボタン1(index0)=前進、ボタン4(index3)=後退
        fwd  = len(msg.buttons) > 0 and msg.buttons[0] == 1
        back = len(msg.buttons) > 3 and msg.buttons[3] == 1

        if fwd:
            for motor_id in self.motor_pins:
                self.move_motor(motor_id, 100)
        elif back:
            for motor_id in self.motor_pins:
                self.move_motor(motor_id, -100)
        else:
            for motor_id in self.motor_pins:
                self.move_motor(motor_id, 0)

    def destroy_node(self):
        self.get_logger().info("ノード終了、すべてのモーターを停止")
        for pwm in self.pwms.values():
            pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MultiMotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
