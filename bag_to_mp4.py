import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class BagToMP4(Node):
    def __init__(self):
        super().__init__('bag_to_mp4')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.video_writer = None
        self.fps = 30  # 수정 가능
        self.output_file = 'output.mp4'
        self.frame_size = None
        self.frame_count = 0

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.video_writer is None:
            height, width, _ = cv_image.shape
            self.frame_size = (width, height)
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(self.output_file, fourcc, self.fps, self.frame_size)
            self.get_logger().info(f'Started writing video to {self.output_file}')

        self.video_writer.write(cv_image)
        self.frame_count += 1

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f'Finished writing {self.frame_count} frames to {self.output_file}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BagToMP4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
