import time
import rclpy
from rclpy.node import Node
from protocol.msg import AudioPlayExtend


class VoiceDetector(Node):
    """
    语音播报模块（轻度优化版）
    功能：调用机器狗的语音模块进行播报
    """
    def __init__(self, node_name='voice_detector', topic_name='/mi_desktop_48_b0_2d_7b_03_9f/speech_play_extend'):
        super().__init__(node_name)
        self.topic_name = topic_name

        # 创建发布器
        self.pub = self.create_publisher(AudioPlayExtend, self.topic_name, 10)
        self.get_logger().info(f"[VoiceDetector] 已创建发布器，topic: {self.topic_name}")

    def wait_for_subscriber(self, timeout=5.0):
        """
        等待订阅者连接，避免消息丢失
        使用非阻塞检查，减少对主线程的影响
        """
        start_time = time.time()
        while rclpy.ok():
            count = self.pub.get_subscription_count()
            if count > 0:
                self.get_logger().info(f"[VoiceDetector] 检测到 {count} 个订阅者已连接")
                return True
            if time.time() - start_time > timeout:
                self.get_logger().warn(f"[VoiceDetector] 等待订阅者超时（>{timeout}s）")
                return False
            rclpy.spin_once(self, timeout_sec=0.05)  # 避免长时间阻塞
        return False

    def play_text(self, text, module_name="warehouse_dog", is_online=True, wait_sub=True):
        """
        发送语音播报请求
        :param text: 要播报的内容
        :param module_name: 模块名（默认 warehouse_dog）
        :param is_online: 是否在线模式
        :param wait_sub: 是否等待订阅者
        """
        if wait_sub:
            self.wait_for_subscriber()

        msg = AudioPlayExtend()
        msg.module_name = module_name
        msg.is_online = is_online
        msg.text = text

        self.pub.publish(msg)
        self.get_logger().info(f"[VoiceDetector] 已发布语音播报: 模块={module_name}, 在线={is_online}, 文本='{text}'")
