import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import traceback

class OdomDistance(Node):
    def __init__(self):
        super().__init__('odom_distance')

        self.get_logger().info("初始化里程计节点...")

        try:
            self.subscription = self.create_subscription(
                Odometry,
                '/mi_desktop_48_b0_2d_7b_03_9f/odom_out',
                self.odom_callback,
                10
            )
            self.get_logger().info("订阅话题 /mi_desktop_48_b0_2d_7b_03_9f/odom_out 成功！")
        except Exception as e:
            self.get_logger().error(f"订阅话题失败: {e}")
            self.get_logger().error(traceback.format_exc())

        self.prev_position = None
        self.total_distance = 0.0

    def quaternion_to_euler_yaw(self, x, y, z, w):
        """
        将四元数转换为偏航角 yaw (弧度)
        """
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def odom_callback(self, msg):
        try:
            # 位置
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z

            # 姿态（四元数转 yaw）
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            yaw = self.quaternion_to_euler_yaw(qx, qy, qz, qw)

            # 打印实时坐标和朝向
            self.get_logger().info(f"坐标: x={x:.3f}, y={y:.3f}, z={z:.3f} | 朝向(yaw)={math.degrees(yaw):.2f}°")

            # 计算累计运动距离
            if self.prev_position is not None:
                dx = x - self.prev_position[0]
                dy = y - self.prev_position[1]
                dz = z - self.prev_position[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                if dist > 5:  # 过滤异常跳变
                    self.get_logger().warn(f"检测到异常跳变: {dist:.3f} m，忽略此段数据")
                else:
                    self.total_distance += dist

            self.prev_position = (x, y, z)
            self.get_logger().info(f"累计运动距离: {self.total_distance:.3f} m")

        except Exception as e:
            self.get_logger().error(f"处理里程计数据时出错: {e}")
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = OdomDistance()
    node.get_logger().info("开始监听里程计数据...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号，准备关闭节点...")
    except Exception as e:
        node.get_logger().error(f"节点运行中发生错误: {e}")
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("节点已关闭。")

if __name__ == '__main__':
    main()

