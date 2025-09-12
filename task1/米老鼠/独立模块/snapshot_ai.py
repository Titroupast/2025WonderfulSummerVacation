#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class SimpleSnapshot(Node):
    def __init__(self):
        super().__init__('simple_snapshot')
        self.bridge = CvBridge()
        self.save_dir = os.path.join(os.getcwd(), "snapshots_auto")
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f"保存目录: {self.save_dir}")

        # 订阅你想要的图像话题，这里以 /image_rgb 为例，请根据你的环境修改
        self.topic_name = '/image'

        self.sub = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            10)

        self.saved = False  # 标记是否已保存，避免重复保存

    def image_callback(self, msg):
        if self.saved:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"转换图像失败: {e}")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = self.topic_name.replace('/', '_').strip('_')
        filename = os.path.join(self.save_dir, f"{safe_name}_{timestamp}.jpg")

        if cv2.imwrite(filename, cv_img):
            self.get_logger().info(f"💾 图像已保存到 {filename}")
            self.saved = True
            rclpy.shutdown()
        else:
            self.get_logger().error(f"❌ 图像保存失败: {filename}")

def main():
    rclpy.init()
    node = SimpleSnapshot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

