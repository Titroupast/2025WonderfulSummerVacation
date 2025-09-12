from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
import cv2
import numpy as np
import time
import rclpy


class ERDetector(Node):
    def __init__(self, template_paths=None, confidence_threshold=0.3):
        super().__init__('er_detector_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        self.er_result = None
        self.text_result = None
        self.last_frame_time = None
        self.last_msg = None
        self.confidence_threshold = confidence_threshold

        # --- 加载数字模板 ---
        if template_paths is None:
            self.template_paths = ['1.jpg', '2.jpg']
        else:
            self.template_paths = template_paths

        self.templates = []
        for path in self.template_paths:
            img = cv2.imread(path)
            if img is None:
                self.get_logger().error(f"无法加载模板文件: {path}")
            else:
                self.templates.append(self.preprocess_image(img))

        self.get_logger().info("ER码 + 数字识别节点已启动")

    # ----------------- 图像预处理 -----------------
    def preprocess_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 15, 4
        )
        return binary

    # ----------------- 回调函数 -----------------
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        barcodes = decode(frame)
        if barcodes:
            data = barcodes[0].data.decode('utf-8')
            self.er_result = data
            self.get_logger().info(f"检测到ER码: {data}")

        self.last_msg = msg
        self.last_frame_time = time.time()

    # ----------------- 文字识别（模板匹配） -----------------
    def detect_digit(self, frame):
        processed_frame = self.preprocess_image(frame)
        detected_text = None
        best_val = 0

        for i, template in enumerate(self.templates):
            result = cv2.matchTemplate(processed_frame, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(result)
            if max_val > best_val and max_val > self.confidence_threshold:
                best_val = max_val
                detected_text = str(i + 1)

        if detected_text:
            self.get_logger().info(f"检测到数字: {detected_text} (置信度: {best_val:.2f})")
        return detected_text

    # ----------------- 检测接口 -----------------
    def detect_er_with_timeout(self, timeout=10.0):
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.er_result:
                return ("ER", self.er_result)

            if time.time() - start_time > timeout:
                self.get_logger().warn("ER码检测超时，尝试文字识别...")

                if self.last_msg is None:
                    self.get_logger().error("没有获取到任何图像帧，无法进行文字识别！")
                    return None

                frame = self.bridge.imgmsg_to_cv2(self.last_msg, desired_encoding='bgr8')
                text_result = self.detect_digit(frame)
                if text_result:
                    return ("TEXT", text_result)
                return None


def main(args=None):
    rclpy.init(args=args)
    node = ERDetector(template_paths=['1.jpg', '2.jpg'])
    result = node.detect_er_with_timeout(timeout=8.0)
    print("检测结果:", result)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
