import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from orca_msg.srv import JpegImage
import subprocess
import os

SIM = False

class CamFbNode(Node):
    def __init__(self):
        super().__init__('cam_fb')

        ####################################################
        # whale cam data as jpg
        self.latest_image = b''

        # subscriber
        self.image_subscription = self.create_subscription(
            CompressedImage,
            '/whale_camera/image_raw/compressed',
            self.image_callback,
            10
        )
        ####################################################

        # service
        self.service = self.create_service(
            JpegImage,
            '/capture',
            self.capture_callback
        )

    ####################################################
    def image_callback(self, msg: CompressedImage):
        self.latest_image = msg.data  # bytes
    ####################################################
    
    def capture_callback(self, request, response):
        response.image = CompressedImage()

        ####################################################
        if SIM:
            if self.latest_image is None:
                self.get_logger().info("No image received yet.")
            else:
                response.image.data = self.latest_image
        ####################################################

        else:
            try:
                result = subprocess.run(
                    ['fswebcam', '-r', '1280x720', '--jpeg', '95', '-D', '1', '--stdout'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=True
                )
                self.get_logger().info("Image captured successfully.")
                response.image.data = result.stdout

            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"image capture failed: {e.stderr.decode().strip()}")
                response.image.data = b''
            
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CamFbNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
