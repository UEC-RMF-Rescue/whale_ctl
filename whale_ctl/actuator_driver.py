import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

from whale_ctl.mecanum_ctl import generate_motor_cmd

class ActuatorDriver(Node):
    def __init__(self):
        super().__init__('actuator_driver')

        # 4 モーターの DIR / PWM ピン定義（ID: {dir, pwm}）
        self.motor_pins = {
            0: {'dir': 8,  'pwm': 27},
            1: {'dir': 18, 'pwm': 12},
            2: {'dir': 25, 'pwm': 16},
            3: {'dir': 26, 'pwm': 20},
        }

        # ランピング設定（速度変化ステップ %）
        self.ramp_step = 5.0
        # 現在のモーター速度を保持 (−100〜100)
        self.current_vel = {motor_id: 0.0 for motor_id in self.motor_pins}

        # GPIO 初期化
        GPIO.setmode(GPIO.BCM)
        self.pwms = {}
        for mid, pins in self.motor_pins.items():
            GPIO.setup(pins['dir'], GPIO.OUT)
            GPIO.setup(pins['pwm'], GPIO.OUT)
            pwm = GPIO.PWM(pins['pwm'], 1000)  # 1kHz PWM
            pwm.start(0)  # 初期 0% 出力
            self.pwms[mid] = pwm
            self.get_logger().info(
                f"Motor {mid} initialized (DIR={pins['dir']}, PWM={pins['pwm']})"
            )

        # /joy トピック購読：ステップ1
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info('Subscribed to /joy — actuator driver ready')

    def joy_callback(self, msg: Joy):
        # ステップ1: Joy axes → v_x, v_y, v_yaw に変換（0〜100）
        v_x   = msg.axes[1] * 100    # 前後: 上 +1 → +100%
        v_y   = msg.axes[0] * 100    # 左右: 右 +1 → +100%
        v_yaw = msg.axes[3] * 100    # 回転: 右 +1 → +100%

        # ステップ2: 行列変換 → 正規化された -1〜1リスト
        wheel_speeds = generate_motor_cmd(v_x, v_y, v_yaw)

        # ステップ3: ランピングしてモーターへ vel 指令
        # step3: move motor according to moto cmd
        for motor_id, norm_vel in enumerate(wheel_speeds):
            # target = norm_vel * 100  # -100〜100
            # curr = self.current_vel[motor_id]
            # diff = target - curr
            # 目標との絶対差に応じてステップ更新
            # if abs(diff) <= self.ramp_step:
            #     curr = target
            # else:
            #     curr += self.ramp_step * (1 if diff > 0 else -1)
            # self.current_vel[motor_id] = curr
            # self.move_motor(motor_id, curr)
            self.move_motor(motor_id, wheel_speeds[motor_id])
            self.get_logger().info("move motor at {wheel_speeds[motor_id]}")
        

    def move_motor(self, motor_id: int, vel: float):
        """
        ステップ3: motor_id のモーターを vel (−100〜100) で駆動
        """
        if motor_id not in self.motor_pins:
            self.get_logger().warn(f"Invalid motor ID: {motor_id}")
            return

        pins = self.motor_pins[motor_id]
        duty = max(0, min(abs(vel), 100))

        # MD10C R3: LOW=前進, HIGH=後退
        GPIO.output(pins['dir'], GPIO.HIGH if vel < 0 else GPIO.LOW)
        self.pwms[motor_id].ChangeDutyCycle(duty)

        # self.get_logger().info(f"[motor{motor_id}] set vel={vel:.1f}, duty={duty}%, dir={'HIGH (reverse)' if vel < 0else 'LOW (forward)'}")
        # self.get_logger().debug(f"[motor{motor_id}] vel={vel:.1f}% → duty={duty:.1f}%")

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

