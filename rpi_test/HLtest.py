import RPi.GPIO as GPIO
import time

# GPIOピン設定
PWM_PIN = 13  # 方向制御ピン
DIR_PIN = 24  # PWM制御ピン（今回はHIGH/LOWのみで使用）

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(PWM_PIN, GPIO.OUT)

    try:
        while True:
            # 正転
            GPIO.output(DIR_PIN, GPIO.LOW)  # 方向ピンをLOWにセット（例：正転）
            GPIO.output(PWM_PIN, GPIO.HIGH) # モーターON（最大出力）
            print("Forward")
            time.sleep(1)

            # 逆転
            GPIO.output(DIR_PIN, GPIO.HIGH) # 方向ピンをHIGHにセット（例：逆転）
            GPIO.output(PWM_PIN, GPIO.HIGH) # モーターON（最大出力）
            print("Backward")
            time.sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        # 終了時にGPIOクリーンアップ
        GPIO.output(PWM_PIN, GPIO.LOW)  # モーターOFF
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    main()

