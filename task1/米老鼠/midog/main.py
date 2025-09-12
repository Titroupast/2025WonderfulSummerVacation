#!/usr/bin/env python3
import numpy as np
import rclpy
import cv2
import time
from act import act

def main(args=None):  
    rclpy.init(args=args)
    ml_sm = None  # 初始化为None
    
    try:
        # 创建状态机实例
        ml_sm = act()
        print("状态机初始化完成，开始运行...")
        
        # 运行ROS节点
        rclpy.spin(ml_sm)
        
    except Exception as e:
        # 捕获并处理异常
        print(f"程序运行出错: {str(e)}")
        if ml_sm:
            ml_sm.get_logger().error(f"严重错误: {str(e)}")
    
    except KeyboardInterrupt:
        # 用户中断处理
        print("程序被用户中断")
        if ml_sm:
            ml_sm.get_logger().info("程序被用户中断")
    
    finally:
        # 确保资源正确释放
        print("正在关闭程序...")
        if ml_sm:
            try:
                # 安全关闭状态机
                ml_sm.shutdown()
                ml_sm.destroy_node()
                print("状态机已安全关闭")
            except Exception as e:
                print(f"关闭过程中出错: {str(e)}")
        
        # 关闭ROS
        rclpy.shutdown()
        print("ROS已关闭")

if __name__ == '__main__':
    # 添加启动延迟，确保系统完全初始化
    time.sleep(1)
    main()