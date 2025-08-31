"""
车牌识别核心模块
提供车牌识别和检测的核心功能，支持图片和视频处理
"""

import os
import re
import time
import base64
import tempfile
import requests
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from typing import Optional, Dict, List, Tuple

class LicensePlateRecognizer:
    """车牌识别器"""
    
    def __init__(self, api_key: str, api_url: str = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"):
        """
        初始化识别器
        
        Args:
            api_key: API密钥
            api_url: API地址
        """
        self.api_key = api_key
        self.api_url = api_url
        self.model_name = "doubao-1-5-vision-pro-32k-250115"
    
    def recognize_plate_number(self, image_bytes: bytes, mime_type: str = "image/jpeg") -> Optional[str]:
        """
        识别车牌号（仅返回车牌号，不检测位置）
        
        Args:
            image_bytes: 图片字节数据
            mime_type: 图片MIME类型
            
        Returns:
            识别出的车牌号，失败返回None
        """
        base64_image = base64.b64encode(image_bytes).decode("utf-8")
        image_data_url = f"data:{mime_type};base64,{base64_image}"
        
        payload = {
            "model": self.model_name,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": image_data_url
                            }
                        },
                        {
                            "type": "text",
                            "text": "请识别图片中的车牌号，只返回车牌号码的字符，不要任何分隔符、空格、标点符号或其他内容。特别注意区分字母D和数字0，字母D有横线，数字0是圆形。例如：皖AD41988，不要写成皖A-D41988或皖A·D02108。"
                        }
                    ]
                }
            ]
        }
        
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        
        try:
            response = requests.post(self.api_url, headers=headers, json=payload)
            response.raise_for_status()
            result = response.json()
            
            if "choices" in result and len(result["choices"]) > 0:
                content = result["choices"][0]["message"]["content"].strip()
                # 清理结果，只保留字母数字字符
                plate_number = ''.join(filter(str.isalnum, content))
                return plate_number if plate_number else None
            else:
                return None
        except Exception as e:
            raise Exception(f"API调用失败: {str(e)}")
    
    def process_image(self, image_path: str) -> Optional[str]:
        """
        处理图片文件，返回识别结果
        
        Args:
            image_path: 图片文件路径
            
        Returns:
            识别出的车牌号，失败返回None
        """
        try:
            # 读取图片
            with open(image_path, 'rb') as f:
                image_bytes = f.read()
            
            # 确保图片尺寸足够大
            image = Image.open(image_path)
            if image.size[0] < 14 or image.size[1] < 14:
                # 如果图片太小，放大到至少100x100
                new_size = (max(100, image.size[0]), max(100, image.size[1]))
                image = image.resize(new_size, Image.Resampling.LANCZOS)
                
                # 重新保存放大后的图片
                with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as temp_file:
                    image.save(temp_file.name, "JPEG", quality=95)
                    with open(temp_file.name, "rb") as f:
                        image_bytes = f.read()
                
                # 清理临时文件
                os.remove(temp_file.name)
            
            # 调用API进行识别
            plate_number = self.recognize_plate_number(image_bytes)
            return plate_number
            
        except Exception as e:
            raise Exception(f"图片处理失败: {str(e)}")
    
    def process_video_frame(self, frame: np.ndarray) -> Optional[str]:
        """
        处理视频帧
        
        Args:
            frame: OpenCV格式的视频帧
            
        Returns:
            识别出的车牌号，失败返回None
        """
        try:
            # 转换为PIL格式
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image_pil = Image.fromarray(frame_rgb)
            
            # 压缩图片以加快API响应
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as temp_img:
                image_pil.save(temp_img.name, quality=80)
                with open(temp_img.name, "rb") as f:
                    img_bytes = f.read()
            
            # 调用API识别
            plate_number = self.recognize_plate_number(img_bytes)
            
            # 删除临时图片
            os.remove(temp_img.name)
            
            return plate_number
            
        except Exception as e:
            return None