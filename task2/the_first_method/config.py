"""
配置文件
管理API密钥、模型参数和其他系统设置
"""

import os
from typing import Dict, Any

class Config:
    """配置管理类"""
    
    # 豆包API配置
    ARK_API_KEY = "YOUR_API_KEY"  # 请替换为您的实际API密钥
    API_URL = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
    MODEL_NAME = "doubao-1-5-vision-pro-32k-250115"
    
    # 评估配置
    DEFAULT_SAMPLE_SIZE = 500
    DEFAULT_TEST_FOLDER = "CCPD2020/CCPD2020/ccpd_green/test"
    
    # 车牌字符映射
    PROVINCES = ["皖", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑", 
                "苏", "浙", "京", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤", 
                "桂", "琼", "川", "贵", "云", "藏", "陕", "甘", "青", "宁", 
                "新", "警", "学", "O"]
    
    ALPHABETS = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L', 
                'M', 'N', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'O']
    
    ADS = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L', 'M', 
           'N', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 
           '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'O']
    
    # GUI配置
    WINDOW_TITLE = "车牌识别系统 - PyQt5版本"
    WINDOW_SIZE = (1200, 800)
    
    # 图片处理配置
    MIN_IMAGE_SIZE = (14, 14)
    RESIZE_TARGET_SIZE = (100, 100)
    JPEG_QUALITY = 95
    TEMP_JPEG_QUALITY = 80
    
    # 视频处理配置
    VIDEO_PROCESS_INTERVAL = 1  # 每秒处理帧数
    MAX_FPS = 10  # 最大显示帧率
    
    # API请求配置
    REQUEST_DELAY = 0.1  # 请求间隔（秒）
    
    @classmethod
    def get_api_config(cls) -> Dict[str, Any]:
        """获取API配置"""
        return {
            'api_key': cls.ARK_API_KEY,
            'api_url': cls.API_URL,
            'model_name': cls.MODEL_NAME
        }
    
    @classmethod
    def validate_config(cls) -> bool:
        """验证配置是否有效"""
        if not cls.ARK_API_KEY or cls.ARK_API_KEY == "YOUR_API_KEY_HERE":
            return False
        return True
    
    @classmethod
    def load_from_env(cls):
        """从环境变量加载配置"""
        cls.ARK_API_KEY = os.getenv('ARK_API_KEY', cls.ARK_API_KEY)
        cls.API_URL = os.getenv('API_URL', cls.API_URL)
        cls.MODEL_NAME = os.getenv('MODEL_NAME', cls.MODEL_NAME)