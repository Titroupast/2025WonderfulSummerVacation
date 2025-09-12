import subprocess
import time

def run_cmd(cmd):
    process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    stdout, stderr = process.communicate()
    if stdout:
        print(stdout)
    if stderr:
        print(stderr)
    return process.returncode

def wait_for_node(node_name, timeout=10):
    """等待 ROS2 节点出现"""
    start = time.time()
    while time.time() - start < timeout:
        ret = subprocess.run(f"ros2 node list | grep {node_name}", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        if ret.stdout.strip() != "":
            return True
        time.sleep(0.5)
    return False

def lifecycle_transition(node, transition):
    """发送 lifecycle 命令前等待节点启动"""
    if wait_for_node(node, timeout=15):
        run_cmd(f"ros2 lifecycle set {node} {transition}")
        print(f"{node} 执行 {transition} 完成")
    else:
        print(f"{node} 未启动，无法执行 {transition}")

def start_all_cameras():
    # 启动总相机
    subprocess.Popen("ros2 launch realsense2_camera on_dog.py", shell=True)
    lifecycle_transition("/camera/camera", "configure")
    lifecycle_transition("/camera/camera", "activate")

    # 启动立体相机
    subprocess.Popen("ros2 run camera_test stereo_camera", shell=True)
    lifecycle_transition("/stereo_camera", "configure")
    lifecycle_transition("/stereo_camera", "activate")

    # 启动 AI 相机服务
    subprocess.Popen("ros2 run camera_test camera_server", shell=True)
    time.sleep(3)
    run_cmd('ros2 service call /camera_service protocol/srv/CameraService "{command: 9, width: 640, height: 480, fps: 0}"')

    # 查看话题
    run_cmd("ros2 topic list | grep image")

if __name__ == "__main__":
    start_all_cameras()
    print("所有相机已启动完成")

