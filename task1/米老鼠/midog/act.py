import math
import cv2
import time
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from Detect_modules.qr_detector import QRDetector
from Detect_modules.voice_detector import VoiceDetector
from Detect_modules.touch_stand_detector import TouchStandModule
from Detect_modules.er_detector import ERDetector

from Detect_modules import YellowLineDetector,ArrowDetector,YellowLightDetector
from Move_modules import  Robot_Ctrl, robot_control_cmd_lcmt

class act(Node):
    def __init__(self):  # 缩进与class对齐
        super().__init__('main_controller')
        self.start_time = 0
        self.bridge = CvBridge()
    
        self.road_lane_detection = YellowLineDetector(y_start=0.0, height_ratio=1.0, width_ratio=1.0)
 
        #self.arrow_detector = ArrowDetector()#箭头检测器
        #self.qr_detector = QRDetector()  # 二维码检测器实例
        self.qr_result = None
        self.er_result=None
        self.current_direction = None  # 当前箭头方向
        self.current_direction_num = 0  # 读取到箭头的次数

        #语音模块
        self.voice_detector = VoiceDetector()
        # self.yellow_light_detector = YellowLightDetector()

        self.lowhead_flag = False
        self.step = 10  # 标定位置

        # self.step=201

        self.last_deviation = 0  # 上一次的偏差值
        self.deviation_history = []  # 偏差历史记录
        self.slope_threshold = 15  # 斜率阈值(像素/帧)
        self.base_speed = 0.2  # 基础前进速度
        self.min_speed = 0.12  # 最小前进速度
        self.max_yaw_speed = 0.8  # 最大偏航速度
        self.slope_gain = 0.5  # 斜率增益系数

        self.Robot_Ctrl = Robot_Ctrl()
        self.Robot_Ctrl.run()
        self.Robot_Ctrl_Msg = robot_control_cmd_lcmt()

        # print("直接站立")
        # self.Robot_Ctrl_Msg.mode = 12  # Recovery stand
        # self.Robot_Ctrl_Msg.gait_id = 0
        # self.Robot_Ctrl_Msg.life_count += 1  # Command will take effect when life_count update
        # self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
        # self.Robot_Ctrl.Wait_finish(12, 0)
        # print("走起来")


        # ROS订阅
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            Image,
            '/image_rgb',
            self.image_callback,
            qos
        )
        print("Subscribed to/image_rgb")

        # 初始化
        self.touch_stand_detector = TouchStandModule(
            touch_topic='/mi_desktop_48_b0_2d_7b_03_9f/touch_status'
        )

        # 状态标志
        self.stand_completed = False

        self.control_timer = self.create_timer(0.05, self.control_loop)

        print("Control timer will start after standing")

        # self.start_time = time.time()

    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def control_loop(self):
        ## if()

        # lf.Robot_Ctrl_Msg.mode = 11
        # self.Robot_Ctrl_Msg.gait_id = 27

        if (self.step == 10):
            self.start()
            self.step = 11
            print("进入step11,A库二维码检测")
            ##
       
        if (self.step == 11):
            self.Robot_Ctrl_Msg.vel_des = [0.0, 0.0, 0.0]
            self.qr_detector = QRDetector()  # 二维码 + 文字识别器实例
            self.get_logger().info("开始二维码/文字识别...")

            result = self.qr_detector.detect_code_with_timeout(timeout=4.0)

            if result:
                code_type, code_value = result
                if code_type == "QR":
                    self.qr_result = code_value
                    self.get_logger().info(f"识别到二维码: {code_value}")
                elif code_type == "TEXT":
                    self.qr_result = code_value
                    self.get_logger().info(f"识别到数字: {code_value}")
            else:
                self.qr_result = None
                self.get_logger().warn("未能识别到二维码或文字")

            # 用完立刻销毁节点，避免一直占用资源
            self.qr_detector.destroy_node()
            self.qr_detector = None

            # ---------- 逻辑分支 ----------
            if self.qr_result == 'A-1' or self.qr_result == '1':
                self.step = 12
                self.get_logger().info("进入step12（A-1区域 / 数字1）")
                self.qr_result = None
                self.enter = 1  # 用来标记初始时进入的装货库
            elif self.qr_result == 'A-2' or self.qr_result == '2':
                self.step = 13
                self.get_logger().info("进入step13（A-2区域 / 数字2）")
                self.qr_result = None
                self.enter = 2
            else:
                # 默认进入 A-1 区域
                self.step = 12
                self.enter = 1


        if self.step in [12, 13]:
            self.qr_result = None

        if self.step == 12:  # 进入A-1右

            #直行
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.2)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.7)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(4)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.1)

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2)

            self.voice_detector.play_text("A 区库位 1")
            
            #阻尼
            self.Robot_Ctrl_Msg.mode=7
            self.Robot_Ctrl_Msg.gait_id=0
            self.Robot_Ctrl_Msg.life_count+=1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            self.Robot_Ctrl.Wait_finish(4,0)
            print("阻尼结束")

# 阻塞等待触摸触发
            if self.touch_stand_detector.wait_for_touch_stand(timeout=30):
               self.get_logger().info("站立动作已触发")
               self.Robot_Ctrl_Msg.mode = 12  # Recovery stand
               self.Robot_Ctrl_Msg.gait_id = 0
               self.Robot_Ctrl_Msg.life_count += 1  # Command will take effect when life_count update
               self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
               self.Robot_Ctrl.Wait_finish(12, 0)

            else:
               self.get_logger().warn("站立等待超时")
            #self.touch_stand_detector.stand_sequence()

            #重新站立
            #self.Robot_Ctrl_Msg.mode = 12  # Recovery stand
            #self.Robot_Ctrl_Msg.gait_id = 0
            #self.Robot_Ctrl_Msg.life_count += 1  # Command will take effect when life_count update
            #self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            #self.Robot_Ctrl.Wait_finish(12, 0)

            # 后退
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [-0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.3)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.3)

            # 左转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.7)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(1.8)

            #回正
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(0.7)

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(15)
            self.step = 20  # 巡线# 巡线   如果画面中  大部分是灰色 就会趴下

        if self.step == 13:  # 进入A-2左

            #直行
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2)
            # 左转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.4)

            # 左转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(4)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(1.9)

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2)

            self.voice_detector.play_text("A 区库位 2")

            #阻尼
            self.Robot_Ctrl_Msg.mode=7
            self.Robot_Ctrl_Msg.gait_id=0
            self.Robot_Ctrl_Msg.life_count+=1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            self.Robot_Ctrl.Wait_finish(4,0)

            #self.touch_stand_detector.stand_sequence()
            # 阻塞等待触摸触发
            if self.touch_stand_detector.wait_for_touch_stand(timeout=30):
               self.get_logger().info("站立动作已触发")
               self.Robot_Ctrl_Msg.mode = 12  # Recovery stand
               self.Robot_Ctrl_Msg.gait_id = 0
               self.Robot_Ctrl_Msg.life_count += 1  # Command will take effect when life_count update
               self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
               self.Robot_Ctrl.Wait_finish(12, 0)

            else:
               self.get_logger().warn("站立等待超时")

            #重新站立
            #self.Robot_Ctrl_Msg.mode = 12  # Recovery stand
            #self.Robot_Ctrl_Msg.gait_id = 0
            #self.Robot_Ctrl_Msg.life_count += 1  # Command will take effect when life_count update
            #self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            #self.Robot_Ctrl.Wait_finish(12, 0)

            # 后退
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [-0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.5)

            # 左转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.5)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(4.3)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.7)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.4, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(1.2)

            # 右转回正
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(0.5)

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(15)
            self.step = 20  # 巡线# 巡线   如果画面中  大部分是灰色 就会趴下
        ##S弯道
        if self.step == 20:
            print("进入S弯道")
            self.S_go()
            self.step=201
            print("S弯道结束")

            #进入箭头识别
        if self.step == 201:
            self.Robot_Ctrl_Msg.vel_des = [0.0, 0.0, 0.0]
            self.arrow_detector = ArrowDetector()#箭头检测器
            self.get_logger().info("开始调用箭头识别模块...")
            # 调用带超时的识别方法，避免机器人卡死
            arrow_result = self.arrow_detector.detect_arrow_with_timeout(timeout=5.0)

            self.arrow_detector.destroy_node()
            self.arrow_detector = None
    
            if arrow_result:

                if arrow_result.lower() == 'right':
                    self.current_direction = 1
                    self.get_logger().info("识别结果为: 右转")
                    self.voice_detector.play_text("右侧路线")
                    self.step = 31
                elif arrow_result.lower() == 'left':
                    self.current_direction = 1
                    self.get_logger().info("识别结果为: 右转")
                    self.voice_detector.play_text("右侧路线")
                    self.step = 32
                else:
                    self.current_direction = None
                    self.get_logger().warn(f"无法识别方向: {arrow_result}")
                    # self.step = 20  # 回到等待状态
            else:
                    self.current_direction = 1
                    self.get_logger().info("未识别到")
                    self.voice_detector.play_text("未识别到，默认右侧路线")
                    self.step = 31


        #从右侧到达二维码
        if self.step == 111:
            self.Robot_Ctrl_Msg.vel_des = [0.0, 0.0, 0.0]
            self.er_detector = ERDetector()  # ER码 + 数字识别器实例
            self.get_logger().info("开始ER码/文字识别...")

            result = self.er_detector.detect_er_with_timeout(timeout=8.0)

            if result:
                code_type, code_value = result
                if code_type == "ER":
                    self.er_result = code_value
                    self.get_logger().info(f"识别到ER码: {code_value}")
                elif code_type == "TEXT":
                    self.er_result = code_value
                    self.get_logger().info(f"识别到数字: {code_value}")
            else:
                self.er_result = None
                self.get_logger().warn("未能识别到ER码或文字")

            # 用完立刻销毁节点
            self.er_detector.destroy_node()
            self.er_detector = None

            # ---------- 逻辑分支 ----------
            if self.er_result == 'B-1' or self.er_result == '1':
                self.step = 1
                self.get_logger().info("进入step1（B-1区域 / 数字1）")
                self.er_result = None
            elif self.er_result == 'B-2' or self.er_result == '2':
                self.step = 2
                self.get_logger().info("进入step2（B-2区域 / 数字2）")
                self.er_result = None
            else:
                # 默认进入 B-2
                self.er_result = 'B-2'
                self.step = 2

        if self.step in[1,4]:
            self.er_result=None       

        #从左侧到达二维码！！！！！！！！
        if self.step == 112:
            self.Robot_Ctrl_Msg.vel_des = [0.0, 0.0, 0.0]
            self.er_detector = ERDetector()  # ER码 + 数字识别器实例
            self.get_logger().info("开始ER码/文字识别...")

            result = self.er_detector.detect_er_with_timeout(timeout=8.0)

            if result:
                code_type, code_value = result
                if code_type == "ER":
                    self.er_result = code_value
                    self.get_logger().info(f"识别到ER码: {code_value}")
                elif code_type == "TEXT":
                    self.er_result = code_value
                    self.get_logger().info(f"识别到数字: {code_value}")
            else:
                self.er_result = None
                self.get_logger().warn("未能识别到ER码或文字")

            # 用完立刻销毁节点
            self.er_detector.destroy_node()
            self.er_detector = None

            # ---------- 逻辑分支 ----------
            if self.er_result == 'B-1' or self.er_result == '1':
                self.step = 3
                self.get_logger().info("进入step3（B-1区域 / 数字1）")
                self.er_result = None
            elif self.er_result == 'B-2' or self.er_result == '2':
                self.step = 4
                self.get_logger().info("进入step4（B-2区域 / 数字2）")
                self.er_result = None
            else:
                # 默认进入 B-1
                self.er_result = 'B-1'
                self.step = 3

         # 右边
        
        #右侧道路
        if self.step == 31: 
            print("右侧道路")
            self.go_right_to_qr()
            self.step=111

        #左侧道路！！！！！！！！！
        if self.step==32:
            print("左侧道路")
            self.go_left_to_qr()
            self.step=112    

        #右侧识别B1
        if self.step==1:
            print("检测到B—1，开始执行路线")
            self.test_B_1()

            # 左转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            #直行
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(5.7)

            # 左转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            #直行
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(16)

            self.stone()

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            #直行
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.66)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(10)


            self.step=123
        #右侧识别B2
        if self.step==2:
            print("检测到B-2，开始执行路线")
            self.test_B_2()
            #旋转180度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(8.1)

            #直行
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(16)

            self.stone()

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            #直行
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.66)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(10)

            self.step=123
        
        
        #左侧识别B1！！！！
        if self.step==3:
            print("检测到B-1,开始执行路线")
            self.test_B_1()

            self.yellow_light_detector = YellowLightDetector()

            # 旋转180度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(7.6)

            self.yellow_light()

            print("即将进入上坡")
                # 旋转180度

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(10)


           # 旋转180度
            self.Robot_Ctrl_Msg.mode = 11
            self.Robot_Ctrl_Msg.gait_id = 26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(8.0)

            self.backward_slope()

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(10)

            # 直行
            self.Robot_Ctrl_Msg.mode = 11
            self.Robot_Ctrl_Msg.gait_id = 26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11
            self.Robot_Ctrl_Msg.gait_id = 26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直行
            self.Robot_Ctrl_Msg.mode = 11
            self.Robot_Ctrl_Msg.gait_id = 26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.66)

            # 左转90度
            self.Robot_Ctrl_Msg.mode = 11
            self.Robot_Ctrl_Msg.gait_id = 26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(10)

            self.step=123
        #左侧识别B2！！！！！！
        if self.step==4:
            print("检测到B-2，开始执行路线")
            self.test_B_2()

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(5.33)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            self.yellow_light_detector = YellowLightDetector()

            self.yellow_light()

            print("即将进入上坡")

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(10)

            # 旋转180度
            self.Robot_Ctrl_Msg.mode = 11
            self.Robot_Ctrl_Msg.gait_id = 26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(7.6)


            self.backward_slope()

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(10)

            # 直行
            self.Robot_Ctrl_Msg.mode = 11
            self.Robot_Ctrl_Msg.gait_id = 26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11
            self.Robot_Ctrl_Msg.gait_id = 26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直行
            self.Robot_Ctrl_Msg.mode = 11
            self.Robot_Ctrl_Msg.gait_id = 26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.66)

            # 左转90度
            self.Robot_Ctrl_Msg.mode = 11
            self.Robot_Ctrl_Msg.gait_id = 26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(10)

            self.step=123
       
        #准备S弯回来
        if self.step==123:
            self.S_back()
            if self.enter==1:#开始去的A1
                self.step=71
            elif self.enter==2:#开始去的A2
                self.step=72    

        #去A2库
        if self.step==71:
             # 左转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(5.33)

             # 左转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.66)

             # 左转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.66)

             # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2)

            self.voice_detector.play_text("到达A 区库位 2")
            
            #阻尼
            self.Robot_Ctrl_Msg.mode=7
            self.Robot_Ctrl_Msg.gait_id=0
            self.Robot_Ctrl_Msg.life_count+=1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            self.Robot_Ctrl.Wait_finish(4,0)
            print("阻尼结束")

# 阻塞等待触摸触发
            if self.touch_stand_detector.wait_for_touch_stand(timeout=30):
               self.get_logger().info("站立动作已触发")
               self.Robot_Ctrl_Msg.mode = 12  # Recovery stand
               self.Robot_Ctrl_Msg.gait_id = 0
               self.Robot_Ctrl_Msg.life_count += 1  # Command will take effect when life_count update
               self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
               self.Robot_Ctrl.Wait_finish(12, 0)

            else:
               self.get_logger().warn("站立等待超时")

            # 后退
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [-0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.06)


             # 左转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

             # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(5.33)

             # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

             # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.6)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)


            # 后退
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [-0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.06)

             # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2)

            #阻尼
            self.Robot_Ctrl_Msg.mode=7
            self.Robot_Ctrl_Msg.gait_id=0
            self.Robot_Ctrl_Msg.life_count+=1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            self.Robot_Ctrl.Wait_finish(4,0)
            print("阻尼结束")

            self.voice_detector.play_text("充电中")
        
        #去A1库
        if self.step==72:
             # 左转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(5.3)

             # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)
            
            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.1)

             # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.7)

            # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2)

            self.voice_detector.play_text("到达A 区库位 1")
            
            #阻尼
            self.Robot_Ctrl_Msg.mode=7
            self.Robot_Ctrl_Msg.gait_id=0
            self.Robot_Ctrl_Msg.life_count+=1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            self.Robot_Ctrl.Wait_finish(4,0)
            print("阻尼结束")

# 阻塞等待触摸触发
            if self.touch_stand_detector.wait_for_touch_stand(timeout=30):
               self.get_logger().info("站立动作已触发")
               self.Robot_Ctrl_Msg.mode = 12  # Recovery stand
               self.Robot_Ctrl_Msg.gait_id = 0
               self.Robot_Ctrl_Msg.life_count += 1  # Command will take effect when life_count update
               self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
               self.Robot_Ctrl.Wait_finish(12, 0)

            else:
               self.get_logger().warn("站立等待超时")

            # 后退
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [-0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.1)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2.6)

             # 左转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 直走
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(4.6)

            # 右转90度
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, -0.5]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.8)

            # 后退
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 26  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [-0.3, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(3.1)

             # 停止
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = 1  # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
            self.Robot_Ctrl_Msg.vel_des = [0, 0, 0]  # 转向
            self.Robot_Ctrl_Msg.duration = 0
            self.Robot_Ctrl_Msg.step_height = [0.06, 0.06]  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            time.sleep(2)

            #阻尼
            self.Robot_Ctrl_Msg.mode=7
            self.Robot_Ctrl_Msg.gait_id=0
            self.Robot_Ctrl_Msg.life_count+=1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
            self.Robot_Ctrl.Wait_finish(4,0)
            print("阻尼结束")

            self.voice_detector.play_text("充电中")

    #开始部分，从充电到二维码检测
    def start(self):
        """机器人起步动作"""

        def send_cmd(vel_x, vel_y, vel_yaw, duration=0, step_height=[0.06, 0.06], gait_id=26):
            """辅助函数，简化发送命令"""
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = gait_id
            self.Robot_Ctrl_Msg.vel_des = [vel_x, vel_y, vel_yaw]  # 转向
            self.Robot_Ctrl_Msg.duration = duration
            self.Robot_Ctrl_Msg.step_height = step_height  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)


        #站立
        print("准备站立")
        self.Robot_Ctrl_Msg.mode = 12  # Recovery stand
        self.Robot_Ctrl_Msg.gait_id = 0
        self.Robot_Ctrl_Msg.life_count += 1  # Command will take effect when life_count update
        self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
        self.Robot_Ctrl.Wait_finish(12, 0)

        ## 第一 小部分
        send_cmd(0.4, 0, 0)
        time.sleep(2.5)

        # 右转90度
        send_cmd(0, 0, -0.5)
        time.sleep(3.8)

        # 直走
        send_cmd(0.4, 0, 0)
        time.sleep(2)

        # 停止
        send_cmd(0, 0, 0, gait_id=1)
        time.sleep(5)     

    def S_go(self):
        """执行一段 S 弯行走动作"""
        
        def send_cmd(vel_x, vel_y, vel_yaw, duration=0, step_height=[0.06, 0.06], gait_id=26):
            """辅助函数，简化发送命令"""
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = gait_id
            self.Robot_Ctrl_Msg.vel_des = [vel_x, vel_y, vel_yaw]
            self.Robot_Ctrl_Msg.duration = duration
            self.Robot_Ctrl_Msg.step_height = step_height
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
        
        # #进行回正
        # send_cmd(0, 0, -0.5)
        # time.sleep(0.8)

        # 直走
        send_cmd(0.3, 0, 0) #转向 
        time.sleep(0.4)

        # 进入S弯 开始走左弯
        send_cmd(0.3, 0, 0.42)
        time.sleep(5.2)

        send_cmd(0.3, 0, 0.52)
        time.sleep(4.6)

        # 直走
        send_cmd(0.3, 0, 0) #转向
        time.sleep(0.3)

        # 第一段右弯
        send_cmd(0.3, 0, -0.43)
        time.sleep(6.0)

        # 人工回正一下
        send_cmd(0.3, 0, -0.54)
        time.sleep(0.3)

        send_cmd(0.3, 0, -0.51)
        time.sleep(5.5)

        # 直走
        send_cmd(0.3, 0, 0) #转向
        time.sleep(0.2)

        # 再走左弯
        send_cmd(0.3, 0, 0.42)
        time.sleep(5.8)

        send_cmd(0.3, 0, 0.52)
        time.sleep(3.1)

        # 直走
        send_cmd(0.3, 0, 0) #转向
        time.sleep(0.8)

        #第二段右弯
        send_cmd(0.3,0,-0.37)
        time.sleep(2.2)

        # 直走
        send_cmd(0.3, 0, 0) #转向
        time.sleep(0.3)

        #第二段右弯
        send_cmd(0.3,0,-0.37)
        time.sleep(2.0)

        # #第二段右弯没走完继续走
        # send_cmd(0.3,0,-0.40)
        # time.sleep(2.0)

        # 直走
        send_cmd(0.3, 0, 0) #转向
        time.sleep(0.6)

        #进行回正
        send_cmd(0, 0, -0.5)
        time.sleep(0.7)

        # 停止
        send_cmd(0, 0, 0, gait_id=1) #转向
        time.sleep(12)
 
    def S_back(self):
        """执行一段 S 弯回退动作"""

        def send_cmd(vel_x, vel_y, vel_yaw, duration=0, step_height=[0.06, 0.06], gait_id=26):
            """辅助函数，简化发送命令"""
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = gait_id
            self.Robot_Ctrl_Msg.vel_des = [vel_x, vel_y, vel_yaw]
            self.Robot_Ctrl_Msg.duration = duration
            self.Robot_Ctrl_Msg.step_height = step_height
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)

        # 初始化站立
        send_cmd(0, 0, 0, gait_id=0)
        time.sleep(0.5)

        # 直走
        send_cmd(0.3, 0, 0)
        time.sleep(0.6)

        # S弯动作（回退方向与S_go相反）
        send_cmd(0, 0, -0.5)
        time.sleep(0.6)

        send_cmd(0.3, 0, 0.42)
        time.sleep(5.0)

        send_cmd(0.3, 0, -0.42)
        time.sleep(5.0)

        send_cmd(0.3, 0, -0.52)
        time.sleep(4.8)

        send_cmd(0.3, 0, 0.43)
        time.sleep(5.3)

        send_cmd(0.3, 0, 0.54)
        time.sleep(0.3)

        send_cmd(0.3, 0, 0.51)
        time.sleep(5.4)

        send_cmd(0.3, 0, 0)
        time.sleep(0.9)

        send_cmd(0.3, 0, -0.40)
        time.sleep(5.0)

        send_cmd(0.3, 0, -0.52)
        time.sleep(4.2)

        send_cmd(0.3, 0, 0)
        time.sleep(0.3)

        send_cmd(0, 0, 0.5)
        time.sleep(0.4)

        send_cmd(0.3, 0, 0)
        time.sleep(1.5)

        send_cmd(0,0,-0.5)
        time.sleep(0.6)

        # 停止
        send_cmd(0, 0, 0, gait_id=1)
        time.sleep(10)

    def stone(self):
        """机器人慢速前进动作"""

        def send_cmd(vel_x, vel_y, vel_yaw, duration=0, step_height=[0.06, 0.06], gait_id=26):
            """辅助函数，简化发送命令"""
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = gait_id
            self.Robot_Ctrl_Msg.vel_des = [vel_x, vel_y, vel_yaw]  # 转向
            self.Robot_Ctrl_Msg.duration = duration
            self.Robot_Ctrl_Msg.step_height = step_height  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)

        # 旋转180°
        send_cmd(0, 0, -0.5)
        time.sleep(8.0)    

        # 前进慢速
        send_cmd(-0.1, 0, 0)
        time.sleep(40)
    
    #走左侧道路去二维码检测！！！！！！！
    def go_left_to_qr(self):
        """机器人行走到二维码位置动作"""

        def send_cmd(vel_x, vel_y, vel_yaw, duration=0, step_height=[0.06, 0.06], gait_id=26):
            """辅助函数，简化发送命令"""
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = gait_id
            self.Robot_Ctrl_Msg.vel_des = [vel_x, vel_y, vel_yaw]  # 转向
            self.Robot_Ctrl_Msg.duration = duration
            self.Robot_Ctrl_Msg.step_height = step_height  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)

        # 直行一部分
        send_cmd(0.3, 0, 0)
        time.sleep(1.8)

        # 左转90度
        send_cmd(0, 0, 0.5)
        time.sleep(3.8)

        # 直行一部分
        send_cmd(0.3, 0, 0)
        time.sleep(3.2)

        # 右转90度
        send_cmd(0, 0, -0.5)
        time.sleep(3.8)

        # 过石板路
        self.stone()

        # 直行（限高杆不要了）
        send_cmd(0.3, 0, 0)
        time.sleep(17)

        # 右转90度
        send_cmd(0, 0, -0.5)
        time.sleep(3.8)

        # 直行
        send_cmd(0.3, 0, 0)
        time.sleep(3.3)

        # 左转90度（看二维码）
        send_cmd(0, 0, 0.5)
        time.sleep(3.8)

        send_cmd(0.0, 0.0, 0.0,gait_id=1)  
        time.sleep(5)
    
    #走右侧道路去二维码检测
    def go_right_to_qr(self): 
        def send_cmd(vel_x, vel_y, vel_yaw, duration=0, step_height=[0.06, 0.06], gait_id=26):
            """辅助函数，简化发送命令"""
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = gait_id
            self.Robot_Ctrl_Msg.vel_des = [vel_x, vel_y, vel_yaw]  # 转向
            self.Robot_Ctrl_Msg.duration = duration
            self.Robot_Ctrl_Msg.step_height = step_height  # ground clearness of swing leg
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)

        # 直行一部分
        send_cmd(0.3, 0, 0)
        time.sleep(1.8)

        # 右转90度
        send_cmd(0, 0, -0.5)
        time.sleep(3.8)

        # 直行一部分
        send_cmd(0.3, 0, 0)
        time.sleep(3.1)

        # 右转90度
        send_cmd(0, 0, -0.5)
        time.sleep(4.1)

        send_cmd(0,0,0,gait_id=1)
        time.sleep(8)
        
        self.backward_slope()

        send_cmd(0,0,0,gait_id=1)
        time.sleep(8)

        self.yellow_light_detector = YellowLightDetector()

        self.yellow_light()

        # 左转90度
        send_cmd(0, 0, 0.5)
        time.sleep(3.8)

        # 直行一部分
        send_cmd(0.3, 0, 0)
        time.sleep(3.2)

        # 右转90度
        send_cmd(0, 0, -0.5)
        time.sleep(3.8)

        #停止
        send_cmd(0.0, 0.0, 0.0,gait_id=1)  
        time.sleep(10)

    def test_B_1(self):

        def send_cmd(vel_x, vel_y, vel_yaw, duration=0, step_height=[0.06, 0.06], gait_id=26, mode=11):
            self.Robot_Ctrl_Msg.mode = mode
            self.Robot_Ctrl_Msg.gait_id = gait_id
            self.Robot_Ctrl_Msg.vel_des = [vel_x, vel_y, vel_yaw]
            self.Robot_Ctrl_Msg.duration = duration
            self.Robot_Ctrl_Msg.step_height = step_height
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)

    # ---- 进入 B-1 库 ----
        send_cmd(0, 0, 0.5)      # 左转90度
        time.sleep(3.8)
        send_cmd(0.3, 0, 0)      # 直走
        time.sleep(3)
        send_cmd(0, 0, -0.5)     # 右转90度
        time.sleep(3.8)
        send_cmd(0.3, 0, 0)      # 直走进库
        time.sleep(3.2)
        send_cmd(0, 0, 0, gait_id=1)  # 停止
        time.sleep(2)

        #播报
        self.voice_detector.play_text("B 区库位 1")

        #阻尼
        self.Robot_Ctrl_Msg.mode=7
        self.Robot_Ctrl_Msg.gait_id=0
        self.Robot_Ctrl_Msg.life_count+=1
        self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
        self.Robot_Ctrl.Wait_finish(4,0)
        print("阻尼结束")

        # 阻塞等待触摸触发
        if self.touch_stand_detector.wait_for_touch_stand(timeout=30):
               self.get_logger().info("站立动作已触发")
               self.Robot_Ctrl_Msg.mode = 12  # Recovery stand
               self.Robot_Ctrl_Msg.gait_id = 0
               self.Robot_Ctrl_Msg.life_count += 1  # Command will take effect when life_count update
               self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
               self.Robot_Ctrl.Wait_finish(12, 0)

        else:
               self.get_logger().warn("站立等待超时")

        send_cmd(-0.4, 0, 0)     # 后退出库
        time.sleep(2.5)

    # ---- 调整方向去 B-2 库 ----
        send_cmd(0, 0, -0.5)     # 右转90度
        time.sleep(3.8)
        send_cmd(0.4, 0, 0)      # 直走
        time.sleep(4.2)
        send_cmd(0, 0, 0.5)     # 左转90度正对B-2库
        time.sleep(3.8)
        send_cmd(0.4, 0, 0)      # 直走进B-2库
        time.sleep(2)
        send_cmd(0, 0, 0, gait_id=1)  # 停止
        time.sleep(2)

        #播报
        self.voice_detector.play_text("B 区库位 2")

        #阻尼
        self.Robot_Ctrl_Msg.mode=7
        self.Robot_Ctrl_Msg.gait_id=0
        self.Robot_Ctrl_Msg.life_count+=1
        self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
        self.Robot_Ctrl.Wait_finish(4,0)

        # 阻塞等待触摸触发
        if self.touch_stand_detector.wait_for_touch_stand(timeout=30):
               self.get_logger().info("站立动作已触发")
               self.Robot_Ctrl_Msg.mode = 12  # Recovery stand
               self.Robot_Ctrl_Msg.gait_id = 0
               self.Robot_Ctrl_Msg.life_count += 1  # Command will take effect when life_count update
               self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
               self.Robot_Ctrl.Wait_finish(12, 0)

        else:
               self.get_logger().warn("站立等待超时")

        send_cmd(-0.4, 0, 0)     # 后退出B-2库
        time.sleep(2.3)

    def test_B_2(self):

        def send_cmd(vel_x, vel_y, vel_yaw, duration=0, step_height=[0.06, 0.06], gait_id=26, mode=11):
            self.Robot_Ctrl_Msg.mode = mode
            self.Robot_Ctrl_Msg.gait_id = gait_id
            self.Robot_Ctrl_Msg.vel_des = [vel_x, vel_y, vel_yaw]
            self.Robot_Ctrl_Msg.duration = duration
            self.Robot_Ctrl_Msg.step_height = step_height
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)

    # ---- 进入 B-2 库 ----
        send_cmd(0, 0, -0.5)      # 右转90度
        time.sleep(3.8)
        send_cmd(0.3, 0, 0)      # 直走
        time.sleep(2.7)
        send_cmd(0, 0, 0.5)     # 左转90度
        time.sleep(3.8)
        send_cmd(0.3, 0, 0)      # 直走进库
        time.sleep(3.4)
        send_cmd(0, 0, 0, gait_id=1)  # 停止
        time.sleep(2)

        #播报
        self.voice_detector.play_text("B 区库位 2")

         #阻尼
        self.Robot_Ctrl_Msg.mode=7
        self.Robot_Ctrl_Msg.gait_id=0
        self.Robot_Ctrl_Msg.life_count+=1
        self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
        self.Robot_Ctrl.Wait_finish(4,0)

        # 阻塞等待触摸触发
        if self.touch_stand_detector.wait_for_touch_stand(timeout=30):
               self.get_logger().info("站立动作已触发")
               self.Robot_Ctrl_Msg.mode = 12  # Recovery stand
               self.Robot_Ctrl_Msg.gait_id = 0
               self.Robot_Ctrl_Msg.life_count += 1  # Command will take effect when life_count update
               self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
               self.Robot_Ctrl.Wait_finish(12, 0)

        else:
               self.get_logger().warn("站立等待超时")

        send_cmd(-0.3, 0, 0)     # 后退出库
        time.sleep(3.4)

    # ---- 调整方向去 B-1 库 ----
        send_cmd(0, 0, 0.5)     # 左转90度
        time.sleep(3.8)
        send_cmd(0.4, 0, 0)      # 直走
        time.sleep(4.3)
        send_cmd(0, 0, -0.5)     # 右转90度正对B-1库
        time.sleep(3.8)
        send_cmd(0.4, 0, 0)      # 直走进B-1库
        time.sleep(2)
        send_cmd(0, 0, 0, gait_id=1)  # 停止
        time.sleep(2)

        #播报
        self.voice_detector.play_text("B 区库位 1")
         #阻尼
        self.Robot_Ctrl_Msg.mode=7
        self.Robot_Ctrl_Msg.gait_id=0
        self.Robot_Ctrl_Msg.life_count+=1
        self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
        self.Robot_Ctrl.Wait_finish(4,0)

        # 阻塞等待触摸触发
        if self.touch_stand_detector.wait_for_touch_stand(timeout=30):
               self.get_logger().info("站立动作已触发")
               self.Robot_Ctrl_Msg.mode = 12  # Recovery stand
               self.Robot_Ctrl_Msg.gait_id = 0
               self.Robot_Ctrl_Msg.life_count += 1  # Command will take effect when life_count update
               self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)
               self.Robot_Ctrl.Wait_finish(12, 0)

        else:
               self.get_logger().warn("站立等待超时")

        send_cmd(-0.4, 0, 0)     # 后退出B-1库
        time.sleep(2)
    
    #上坡
    def backward_slope(self):
        """执行倒着上坡，再旋转恢复方向，最后直行"""

        def send_cmd(vel_x, vel_y, vel_yaw, duration=0, step_height=[0.06, 0.06], gait_id=26):
            """辅助函数，简化发送命令"""
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = gait_id
            self.Robot_Ctrl_Msg.vel_des = [vel_x, vel_y, vel_yaw]
            self.Robot_Ctrl_Msg.duration = duration
            self.Robot_Ctrl_Msg.step_height = step_height
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)

        # # 第一次旋转180°
        # send_cmd(0, 0, -0.5)
        # time.sleep(8.1)

        # 倒着上坡
        send_cmd(-0.1, 0, 0)
        time.sleep(40)

        # # 第二次旋转180°（回正）
        # send_cmd(0, 0, -1)
        # time.sleep(3.8)

        # # 直行
        # send_cmd(0.2, 0, 0)
        # time.sleep(13.5)

        # 上完坡之后回正旋转180°
        send_cmd(0, 0, 0.5)
        time.sleep(8.1)
        # 停止
        send_cmd(0, 0, 0, gait_id=1)
        time.sleep( 3 )
        print("完成倒着上坡动作")
 
    #黄灯检测
    def yellow_light(self):
        """遇到黄灯，检测圆形并精确控制停车和前进"""

        def send_cmd(vel_x, vel_y, vel_yaw, duration=0, step_height=[0.06, 0.06], gait_id=26):
            """辅助函数，简化发送命令"""
            self.Robot_Ctrl_Msg.mode = 11  # Locomotion
            self.Robot_Ctrl_Msg.gait_id = gait_id
            self.Robot_Ctrl_Msg.vel_des = [vel_x, vel_y, vel_yaw]
            self.Robot_Ctrl_Msg.duration = duration
            self.Robot_Ctrl_Msg.step_height = step_height
            self.Robot_Ctrl_Msg.life_count += 1
            self.Robot_Ctrl.Send_cmd(self.Robot_Ctrl_Msg)

        # 开始计时
        # self.start_time = time.time()

        # 初始直行
        send_cmd(0.2, 0, 0, gait_id=26)
        time.sleep(7.5)
        print("开始黄灯检测与通过动作")
        # circle_area = None
        # while True:
        #     # 每 0.1 秒更新一次黄灯检测结果
        #     rclpy.spin_once(self.yellow_light_detector, timeout_sec=0.1)

        #     if self.yellow_light_detector.circle_area:
        #         circle_area = self.yellow_light_detector.circle_area
        #         self.get_logger().info(f"实时检测到黄灯面积: {circle_area:.2f}")

        #         # 达到阈值时触发动作
        #         if 8000< circle_area :
        #             self.end_time = time.time()

        #             # 停止
        #             send_cmd(0, 0, 0, gait_id=1)
        #             time.sleep(10)

        #             # 计算行驶距离
        #             self.elapsed_time = self.end_time - self.start_time
        #             self.distance = 0.2 * self.elapsed_time
        
        send_cmd(0, 0, 0, gait_id=1)
        time.sleep(4)

        self.voice_detector.play_text("5    4    3    2    1")

                    # # 计算剩余距离和对应的行驶时间
                    # remain_distance = 4.5 - self.distance
                    # move_time = remain_distance / 0.2

        send_cmd(0,0,0,gait_id=1)
        time.sleep(5)            
         

        # 继续直行
        send_cmd(0.2, 0, 0, gait_id=26)
        time.sleep(15.2)

                    # 最终停止
        send_cmd(0, 0, 0, gait_id=1)
        time.sleep(8)

            #         break  # 结束循环，完成流程

            # # 防止死循环，可以加个超时（例如 20 秒）
            # if time.time() - self.start_time > 20:
            #     self.get_logger().warn("黄灯检测超时，未触发停车条件")
            #     break

        print("完成黄灯检测与通过动作")


    def shutdown(self):
        self.Robot_Ctrl.quit()   # 停止机器人动作
        cv2.destroyAllWindows()  # 销毁所有 OpenCV 窗口
        self.get_logger().info("Multi_level_State_Machine 已关闭")
   