import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
import cv2
import numpy as np
import time


class QRDetector(Node):
    def __init__(self, template_paths=None, confidence_threshold=0.3):
        super().__init__('qr_detector_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/image_rgb',
            self.image_callback,
            10
        )

        # --- 状态变量 ---
        self.qr_result = None
        self.text_result = None
        self.last_frame_time = None
        self.confidence_threshold = confidence_threshold

        # --- 加载文字模板 ---
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

        self.get_logger().info("二维码+文字识别节点已启动")

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

        # 先尝试二维码识别
        barcodes = decode(frame)
        if barcodes:
            data = barcodes[0].data.decode('utf-8')
            self.qr_result = data
            self.get_logger().info(f"检测到二维码: {data}")
        else:
            # 如果没有二维码，再尝试文字识别
            processed_frame = self.preprocess_image(frame)
            detected_text = None
            best_val = 0

            for i, template in enumerate(self.templates):
                result = cv2.matchTemplate(processed_frame, template, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(result)
                if max_val > best_val and max_val > self.confidence_threshold:
                    best_val = max_val
                    detected_text = str(i + 1)

            if detected_text:
                self.text_result = detected_text
                self.get_logger().info(f"检测到数字: {detected_text} (置信度: {best_val:.2f})")

        self.last_frame_time = time.time()

    # ----------------- 检测接口 -----------------
    def detect_code_with_timeout(self, timeout=10.0):
        """
        先检测二维码，如果失败再检测文字
        """
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.qr_result:
                return ("QR", self.qr_result)

            if self.text_result:
                return ("TEXT", self.text_result)

            if time.time() - start_time > timeout:
                self.get_logger().warn("二维码和文字检测超时")
                return None


# ----------------- 测试 -----------------
def main(args=None):
    rclpy.init(args=args)
    node = QRDetector(template_paths=['1.jpg', '2.jpg'])
    result = node.detect_code_with_timeout(timeout=10.0)
    print("检测结果:", result)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
