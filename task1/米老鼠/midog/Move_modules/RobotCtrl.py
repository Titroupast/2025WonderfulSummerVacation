import lcm

import time
from threading import Thread, Lock

from .robot_control_cmd_lcmt import robot_control_cmd_lcmt
from .robot_control_response_lcmt import robot_control_response_lcmt
from .localization_lcmt import localization_lcmt

class Robot_Ctrl(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_response)
        self.send_thread = Thread(target=self.send_publish)
        self.rec_thread_rpy = Thread(target=self.rec_response_rpy)

        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.lc_rrpy = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")

        self.rec_msgrpy = localization_lcmt()
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()

        self.send_lock = Lock()
        self.roll = 0.0         
        self.pitch = 0.0
        self.yaw = 0.0
        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.runing = 1


    def run(self):
        
        self.lc_r.subscribe("robot_control_response", self.Robot_msg_handler)
        self.lc_rrpy.subscribe("global_to_robot", self.RPY_msg_handler)  
        # 启动所有线程
        self.send_thread.start()
        self.rec_thread.start()
        self.rec_thread_rpy.start()  
    

    def RPY_msg_handler(self, channel, data):
        self.rec_msgrpy = localization_lcmt().decode(data) 
        self.roll = self.rec_msgrpy.rpy[0]
        self.pitch = self.rec_msgrpy.rpy[1]
        self.yaw = self.rec_msgrpy.rpy[2]


    def rec_response_rpy(self):
        while self.runing:
            self.lc_rrpy.handle() 
            time.sleep(0.05)

    def get_rpy(self):
        return (self.roll, self.pitch, self.yaw)

    def Robot_msg_handler(self, channel, data):
        self.rec_msg = robot_control_response_lcmt().decode(data)
        if(self.rec_msg.order_process_bar >= 95):
            self.mode_ok = self.rec_msg.mode
        else:
            self.mode_ok = 0

    def rec_response(self):
        while self.runing:
            self.lc_r.handle()
            time.sleep( 0.002 )

    def Wait_finish(self, mode, gait_id):
        count = 0
        while self.runing and count < 2000: #10s
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1

    def send_publish(self):
        while self.runing:
            self.send_lock.acquire()
            if self.delay_cnt > 20: # Heartbeat signal 10HZ, It is used to maintain the heartbeat when life count is not updated
                self.lc_s.publish("robot_control_cmd",self.cmd_msg.encode())
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep( 0.005 )
            if(self.cmd_msg.life_count > 100):
                self.cmd_msg.life_count = 0

    def Send_cmd(self, msg):
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.send_lock.release()

    def quit(self):
        self.runing = 0
        self.rec_thread.join()
        self.send_thread.join()
