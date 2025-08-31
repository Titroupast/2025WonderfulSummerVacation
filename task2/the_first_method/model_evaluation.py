"""
模型评估模块
提供车牌识别模型的性能评估功能，包括准确率、响应时间、错误分析等指标
"""

import os
import time
import json
import random
import requests
import base64
from datetime import datetime
from typing import List, Dict, Tuple, Optional
import pandas as pd
import numpy as np
from PIL import Image
import cv2

class LicensePlateEvaluator:
    """车牌识别模型评估器"""
    
    def __init__(self, api_key: str, api_url: str = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"):
        """
        初始化评估器
        
        Args:
            api_key: API密钥
            api_url: API地址
        """
        self.api_key = api_key
        self.api_url = api_url
        self.model_name = "doubao-1-5-vision-pro-32k-250115"
        
        # 车牌字符映射表
        self.provinces = ["皖", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑", 
                         "苏", "浙", "京", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤", 
                         "桂", "琼", "川", "贵", "云", "藏", "陕", "甘", "青", "宁", 
                         "新", "警", "学", "O"]
        self.alphabets = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L', 
                         'M', 'N', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'O']
        self.ads = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L', 'M', 
                   'N', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 
                   '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'O']
    
    def parse_filename(self, filename: str) -> Optional[str]:
        """
        从文件名解析真实车牌号
        
        Args:
            filename: 文件名
            
        Returns:
            解析出的车牌号，如果解析失败返回None
        """
        name = filename.split('.')[0]
        parts = name.split('-')
        
        # 查找包含车牌字符索引的部分
        for part in parts:
            if '_' in part and len(part.split('_')) >= 7:
                subparts = part.split('_')
                if len(subparts) >= 7:
                    try:
                        # 尝试解析所有可能的数字作为车牌字符索引
                        numbers = []
                        for subpart in subparts:
                            try:
                                num = int(subpart)
                                numbers.append(num)
                            except ValueError:
                                break
                        
                        # 如果找到足够的数字，尝试解析车牌
                        if len(numbers) >= 7:
                            # 尝试8个字符的情况（如果有8个数字）
                            if len(numbers) >= 8:
                                try:
                                    province = self.provinces[numbers[0]] if numbers[0] < len(self.provinces) else "O"
                                    alphabet = self.alphabets[numbers[1]] if numbers[1] < len(self.alphabets) else "O"
                                    ad_chars = [self.ads[idx] if idx < len(self.ads) else "O" for idx in numbers[2:8]]
                                    plate_number = province + alphabet + ''.join(ad_chars)
                                    return plate_number
                                except (IndexError, ValueError):
                                    pass
                            
                            # 尝试7个字符的情况
                            try:
                                province = self.provinces[numbers[0]] if numbers[0] < len(self.provinces) else "O"
                                alphabet = self.alphabets[numbers[1]] if numbers[1] < len(self.alphabets) else "O"
                                ad_chars = [self.ads[idx] if idx < len(self.ads) else "O" for idx in numbers[2:7]]
                                plate_number = province + alphabet + ''.join(ad_chars)
                                return plate_number
                            except (IndexError, ValueError):
                                pass
                                
                    except (ValueError, IndexError):
                        continue
        return None
    
    def recognize_plate_number(self, image_bytes: bytes, mime_type: str = "image/jpeg") -> Optional[str]:
        """
        调用API识别车牌号
        
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
    
    def calculate_accuracy(self, predicted: str, actual: str) -> float:
        """
        计算车牌识别准确率（严格匹配）
        
        Args:
            predicted: 预测的车牌号
            actual: 实际的车牌号
            
        Returns:
            准确率（0.0或1.0）
        """
        if not predicted or not actual:
            return 0.0
        
        # 去除所有非字母数字字符
        predicted_clean = ''.join(filter(str.isalnum, predicted))
        actual_clean = ''.join(filter(str.isalnum, actual))
        
        # 严格匹配：完全相同返回1.0，否则返回0.0
        return 1.0 if predicted_clean == actual_clean else 0.0
    
    def calculate_character_accuracy(self, predicted: str, actual: str) -> float:
        """
        计算字符级别准确率
        
        Args:
            predicted: 预测的车牌号
            actual: 实际的车牌号
            
        Returns:
            字符级别准确率（0.0-1.0）
        """
        if not predicted or not actual:
            return 0.0
        
        predicted_clean = ''.join(filter(str.isalnum, predicted))
        actual_clean = ''.join(filter(str.isalnum, actual))
        
        if len(predicted_clean) != len(actual_clean):
            return 0.0
        
        if len(actual_clean) == 0:
            return 0.0
        
        correct_chars = sum(1 for p, a in zip(predicted_clean, actual_clean) if p == a)
        return correct_chars / len(actual_clean)
    
    def evaluate_batch(self, test_folder: str, sample_size: int = 500, 
                      progress_callback=None) -> List[Dict]:
        """
        批量评估模型性能
        
        Args:
            test_folder: 测试图片文件夹路径
            sample_size: 测试样本数量
            progress_callback: 进度回调函数
            
        Returns:
            评估结果列表
        """
        if not os.path.exists(test_folder):
            raise FileNotFoundError(f"测试文件夹不存在: {test_folder}")
        
        # 获取所有图片文件
        image_files = [f for f in os.listdir(test_folder) 
                      if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
        
        if len(image_files) < sample_size:
            sample_size = len(image_files)
        
        # 随机选择图片
        selected_files = random.sample(image_files, sample_size)
        
        results = []
        
        for i, filename in enumerate(selected_files):
            if progress_callback:
                progress_callback(i + 1, sample_size, filename)
            
            # 解析真实车牌号
            actual_plate = self.parse_filename(filename)
            
            if not actual_plate:
                continue
            
            # 读取图片
            image_path = os.path.join(test_folder, filename)
            with open(image_path, 'rb') as f:
                image_bytes = f.read()
            
            # 记录开始时间
            start_time = time.time()
            
            try:
                # 调用API识别
                predicted_plate = self.recognize_plate_number(image_bytes)
                
                # 记录结束时间
                end_time = time.time()
                response_time = end_time - start_time
                
                # 计算准确率
                strict_accuracy = self.calculate_accuracy(predicted_plate, actual_plate)
                char_accuracy = self.calculate_character_accuracy(predicted_plate, actual_plate)
                predicted_clean = ''.join(filter(str.isalnum, predicted_plate)) if predicted_plate else ""
                
                results.append({
                    'filename': filename,
                    'actual_plate': actual_plate,
                    'predicted_plate': predicted_plate,
                    'predicted_clean': predicted_clean,
                    'strict_accuracy': strict_accuracy,
                    'char_accuracy': char_accuracy,
                    'response_time': response_time,
                    'is_correct': strict_accuracy == 1.0,
                    'success': True
                })
                
            except Exception as e:
                results.append({
                    'filename': filename,
                    'actual_plate': actual_plate,
                    'predicted_plate': None,
                    'predicted_clean': "",
                    'strict_accuracy': 0.0,
                    'char_accuracy': 0.0,
                    'response_time': time.time() - start_time,
                    'is_correct': False,
                    'success': False,
                    'error': str(e)
                })
            
            # 添加延迟避免API限制
            time.sleep(0.1)
        
        return results
    
    def calculate_statistics(self, results: List[Dict]) -> Dict:
        """
        计算评估统计数据
        
        Args:
            results: 评估结果列表
            
        Returns:
            统计数据字典
        """
        if not results:
            return {}
        
        df = pd.DataFrame(results)
        
        total_images = len(df)
        successful_recognitions = len(df[df['success'] == True])
        correct_recognitions = len(df[df['is_correct'] == True])
        
        stats = {
            'total_images': total_images,
            'successful_recognitions': successful_recognitions,
            'correct_recognitions': correct_recognitions,
            'success_rate': successful_recognitions / total_images * 100,
            'strict_accuracy': correct_recognitions / total_images * 100,
            'avg_char_accuracy': df['char_accuracy'].mean() * 100,
            'avg_response_time': df['response_time'].mean(),
            'min_response_time': df['response_time'].min(),
            'max_response_time': df['response_time'].max(),
            'std_response_time': df['response_time'].std(),
            'error_count': len(df[df['success'] == False])
        }
        
        return stats
    
    def save_results(self, results: List[Dict], output_file: str = None) -> str:
        """
        保存评估结果到文件
        
        Args:
            results: 评估结果列表
            output_file: 输出文件路径，如果为None则自动生成
            
        Returns:
            保存的文件路径
        """
        if output_file is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_file = f"evaluation_results_{timestamp}.json"
        
        # 添加统计信息
        stats = self.calculate_statistics(results)
        output_data = {
            'timestamp': datetime.now().isoformat(),
            'statistics': stats,
            'results': results
        }
        
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, ensure_ascii=False, indent=2)
        
        return output_file
    
    def get_error_analysis(self, results: List[Dict], top_n: int = 10) -> List[Dict]:
        """
        获取错误案例分析
        
        Args:
            results: 评估结果列表
            top_n: 返回的错误案例数量
            
        Returns:
            错误案例列表
        """
        error_cases = [r for r in results if r['strict_accuracy'] < 1.0]
        return error_cases[:top_n]