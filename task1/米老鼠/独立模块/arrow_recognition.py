import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ArrowDetector(Node):
    """箭头检测节点，基于质心像素统计判断方向"""
    def __init__(self, debug=False):
        super().__init__('arrow_detector_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )
        self.arrow_result = None
        self.last_frame_time = None
        self.debug = debug
        self.get_logger().info("箭头检测节点已启动")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        direction = self.detect_arrow_direction(frame)
        if direction:
            self.arrow_result = direction
            self.get_logger().info(f"检测到箭头方向: {direction}")
        self.last_frame_time = time.time()

    def detect_arrow_direction(self, img, crop_ratio=0.4, start_ratio=0.3):
        h, w = img.shape[:2]
        crop_w = int(w * crop_ratio)
        crop_h = int(h * crop_ratio)
        x_start = (w - crop_w) // 2
        y_start = int(h * start_ratio)
        if y_start + crop_h > h:
            crop_h = h - y_start
        cropped = img[y_start:y_start + crop_h, x_start:x_start + crop_w]

        # HSV + 绿色掩膜
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        masked = cv2.bitwise_and(cropped, cropped, mask=mask)

        # 灰度 + 二值化
        gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

        # 找最大轮廓
        contours_info = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)

        # 判断方向
        direction = self.get_arrow_direction_by_centroid(binary, largest)
        return direction

    @staticmethod
    def get_arrow_direction_by_centroid(binary, contour):
        """
        通过质心左右的像素点数量判断箭头方向（适用于实心箭头）。
        """
        M = cv2.moments(contour)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])

        # 直接使用二值图的白色像素（255）进行统计
        ys, xs = np.where(binary == 255)
        left_count = np.sum(xs < cx)
        right_count = np.sum(xs > cx)

        # 输出调试信息（可选）
        # print(f"左侧像素数: {left_count}, 右侧像素数: {right_count}")

        return "left" if left_count > right_count else "right"


# 删除 detect_arrow_with_timeout 的循环阻塞版本
# 改为：
    def get_latest_arrow(self):
       return self.arrow_result


def main(args=None):
    rclpy.init(args=args)
    node = ArrowDetector(debug=False)
    result = node.detect_arrow_with_timeout(timeout=10.0)
    print("检测结果:", result)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
