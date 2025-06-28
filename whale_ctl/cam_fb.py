import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from orca_msg.srv import JpegImage
import subprocess
import os
from ament_index_python.packages import get_package_share_directory

SIM = False

class CamFbNode(Node):
    def __init__(self):
        super().__init__('cam_fb')

        # whale cam data as jpg
        self.latest_image = b''
        self.img_name = "latest_image.jpg"

        ####################################################
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

        pkg_path = get_package_share_directory('whale_ctl')
        img_dir = os.path.join(pkg_path, 'image')
        self.img_path = os.path.join(img_dir, self.img_name)

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
                    ['fswebcam', '-r', '1280x720', '--jpeg', '95', '-D', '1', self.img_path],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    check=True
                )

                self.get_logger().info("Image captured successfully.")
                # read as binary
                with open(self.img_path, 'rb') as f:
                    response.image.data = f.read

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
