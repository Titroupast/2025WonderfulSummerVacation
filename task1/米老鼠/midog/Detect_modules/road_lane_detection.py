import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class YellowLineDetector:
    def __init__(self, y_start=0.0, height_ratio=1.0, width_ratio=1.0):
        self.y_start = y_start
        self.height_ratio = height_ratio
        self.width_ratio = width_ratio
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([30, 255, 255])
        self.min_contour_area = 50

    def set_size(self, y_start, height_ratio=1.0, width_ratio=1.0):
        self.y_start = y_start
        self.height_ratio = height_ratio
        self.width_ratio = width_ratio

    def process_image(self, frame, flag=False):
        cropped = self._crop_roi(frame)
        S_isfinal = self.detect_S_final(cropped) if flag else False
        mask, contour, center = self._detect_yellow_line(cropped)
        if center is None:
            return None, mask, contour, S_isfinal
        deviation = self._calculate_deviation(center, cropped.shape[1])
        return deviation, mask, contour, S_isfinal

    def detect_S_final(self, image, n_clusters=2, threshold=0.95):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0], None, [180], [0, 180])
        max_val = np.max(hist)
        ratio = max_val / np.sum(hist)
        return ratio >= threshold

    def _crop_roi(self, image):
        h, w = image.shape[:2]
        y = int(h * self.y_start)
        crop_h = int(h * self.height_ratio)
        crop_w = int(w * self.width_ratio)
        x = int((w - crop_w) / 2)
        y = max(0, min(h-1, y))
        crop_h = max(1, min(h - y, crop_h))
        x = max(0, min(w-1, x))
        crop_w = max(1, min(w - x, crop_w))
        return image[y:y+crop_h, x:x+crop_w]

    def _detect_yellow_line(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        kernel = np.ones((3, 3), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        contours = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if not contours:
            return yellow_mask, None, None
        max_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(max_contour) < self.min_contour_area:
            return yellow_mask, None, None
        M = cv2.moments(max_contour)
        if M["m00"] == 0:
            return yellow_mask, None, None
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return yellow_mask, max_contour, (cX, cY)

    def _calculate_deviation(self, center, width):
        return center[0] - (width // 2)


# 节点类单独分离，只在直接运行时启动
class YellowLineDetectorNode(Node):
    def __init__(self):
        super().__init__('yellow_line_detector_node')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/image_right',  # 根据实际topic调整
            self.image_callback,
            10
        )

        # 初始化黄线检测器
        self.detector = YellowLineDetector(y_start=0.0, height_ratio=1.0, width_ratio=1.0)
        self.deviation = None
        self.S_isfinal = False

        self.get_logger().info("黄线检测节点已启动")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 调用黄线检测模块
        flag_1 = False  # 可根据逻辑设置 S弯检测开关
        self.deviation, _, _, self.S_isfinal = self.detector.process_image(frame, flag=flag_1)

        # 可打印或发布偏差信息
        if self.deviation is not None:
            self.get_logger().info(f"道路偏差: {self.deviation}, S弯: {self.S_isfinal}")


def main(args=None):
    rclpy.init(args=args)
    node = YellowLineDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()