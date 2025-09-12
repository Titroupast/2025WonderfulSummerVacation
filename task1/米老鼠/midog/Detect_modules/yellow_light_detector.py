import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class YellowLightDetector(Node):
    """黄灯检测节点，检测图像中面积最大的黄灯并返回其圆形面积"""
    def __init__(self):
        super().__init__('yellow_light_detector_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )
        self.circle_area = None  # 检测到的最大黄灯圆面积
        self.last_frame_time = None
        self.get_logger().info("黄灯检测节点已启动")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, _, _, circle_area = self.detect_yellow_light(frame)
        if circle_area:
            self.circle_area = circle_area
            self.get_logger().info(f"检测到黄灯，圆面积: {circle_area:.2f}")
        self.last_frame_time = time.time()

    def detect_yellow_light(self, image):
        """检测图像中的黄灯，返回绘制结果、边界框列表、矩形面积、圆面积"""
        output_image = image.copy()
        identified_lights = []

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([17, 150, 100])
        upper_yellow = np.array([25, 255, 140])
        mask_color = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 开运算
        kernel_open = np.ones((3, 3), np.uint8)
        mask_open = cv2.morphologyEx(mask_color, cv2.MORPH_OPEN, kernel_open)

        # 闭运算
        kernel_close = np.ones((5, 5), np.uint8)
        mask_final = cv2.morphologyEx(mask_open, cv2.MORPH_CLOSE, kernel_close)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_final, 8, cv2.CV_32S)

        img_height, img_width = image.shape[:2]

        max_area = 0
        max_light = None
        rect_area = 0
        circle_area = 0

        for i in range(1, num_labels):
            x, y, w, h, area = stats[i]
            aspect_ratio = w / float(h) if h > 0 else 0
            if area < 100 or aspect_ratio < 0.5 or aspect_ratio > 2.0:
                continue
            if img_height * 0.1 <= y + h / 2 <= img_height * 0.9:
                if area > max_area:
                    max_area = area
                    max_light = (x, y, w, h)
                    rect_area = w * h
                    center = (int(x + w / 2), int(y + h / 2))
                    radius = int(max(w, h) / 2)
                    circle_area = np.pi * radius * radius

        if max_light:
            identified_lights.append(max_light)

        return output_image, identified_lights, rect_area, circle_area

    def detect_yellow_with_timeout(self, timeout=10.0):
        """阻塞等待黄灯检测结果（返回 circle_area）"""
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.circle_area is not None:
                return self.circle_area
            if time.time() - start_time > timeout:
                self.get_logger().warn("黄灯检测超时")
                return None

def main(args=None):
    rclpy.init(args=args)
    node = YellowLightDetector()
    circle_area = node.detect_yellow_with_timeout(timeout=10.0)
    print("检测到黄灯圆面积:", circle_area)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
