"""
基于PyQt5的车牌识别GUI应用
替代Streamlit，提供更好的用户体验和界面控制
"""

import sys
import os
import json
import tempfile
import cv2
import numpy as np
from datetime import datetime
from typing import Optional, List, Dict

from PyQt5.QtWidgets import (QApplication, QMainWindow, QTabWidget, QWidget, 
                            QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
                            QFileDialog, QTextEdit, QProgressBar, QSpinBox,
                            QTableWidget, QTableWidgetItem, QMessageBox,
                            QGroupBox, QGridLayout, QSplitter, QScrollArea)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QFont, QIcon

from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import seaborn as sns

from model_evaluation import LicensePlateEvaluator
from license_plate_recognition import LicensePlateRecognizer
from config import Config

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class RecognitionWorker(QThread):
    """图片识别工作线程"""
    finished = pyqtSignal(dict)
    error = pyqtSignal(str)
    
    def __init__(self, image_path, api_key):
        super().__init__()
        self.image_path = image_path
        self.api_key = api_key
        self.recognizer = LicensePlateRecognizer(api_key)
    
    def run(self):
        try:
            # 处理图片并获取识别结果
            plate_number = self.recognizer.process_image(self.image_path)
            
            self.finished.emit({
                'plate_number': plate_number,
                'image_path': self.image_path
            })
        except Exception as e:
            self.error.emit(str(e))

class VideoWorker(QThread):
    """视频处理工作线程"""
    frame_processed = pyqtSignal(np.ndarray, str)  # 发送帧和车牌号
    finished = pyqtSignal()
    error = pyqtSignal(str)
    
    def __init__(self, video_path, api_key):
        super().__init__()
        self.video_path = video_path
        self.api_key = api_key
        self.recognizer = LicensePlateRecognizer(api_key)
        self.running = False
    
    def run(self):
        try:
            cap = cv2.VideoCapture(self.video_path)
            frame_rate = cap.get(cv2.CAP_PROP_FPS)
            interval = max(1, int(frame_rate))  # 每秒识别一帧
            frame_count = 0
            last_plate_number = ""
            
            self.running = True
            
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    break
                
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                if frame_count % interval == 0:
                    # 处理当前帧进行识别
                    plate_number = self.recognizer.process_video_frame(frame)
                    if plate_number:
                        last_plate_number = plate_number
                
                # 发送处理后的帧和当前识别结果
                self.frame_processed.emit(frame_rgb, last_plate_number)
                frame_count += 1
                
                # 控制播放速度
                self.msleep(int(1000 / max(10, frame_rate)))
            
            cap.release()
            self.finished.emit()
            
        except Exception as e:
            self.error.emit(str(e))
    
    def stop(self):
        """停止视频处理"""
        self.running = False

class EvaluationWorker(QThread):
    """批量评估工作线程"""
    progress = pyqtSignal(int, int, str)  # current, total, filename
    finished = pyqtSignal(list)
    error = pyqtSignal(str)
    
    def __init__(self, test_folder, sample_size, api_key):
        super().__init__()
        self.test_folder = test_folder
        self.sample_size = sample_size
        self.api_key = api_key
        self.evaluator = LicensePlateEvaluator(api_key)
    
    def run(self):
        try:
            def progress_callback(current, total, filename):
                self.progress.emit(current, total, filename)
            
            results = self.evaluator.evaluate_batch(
                self.test_folder, 
                self.sample_size, 
                progress_callback
            )
            self.finished.emit(results)
        except Exception as e:
            self.error.emit(str(e))

class RecognitionTab(QWidget):
    """车牌识别功能页面"""
    
    def __init__(self, api_key):
        super().__init__()
        self.api_key = api_key
        self.current_file_path = None
        self.video_worker = None
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # 文件选择区域
        file_group = QGroupBox("文件选择")
        file_layout = QHBoxLayout()
        
        self.select_image_btn = QPushButton("选择图片")
        self.select_image_btn.clicked.connect(self.select_image)
        
        self.select_video_btn = QPushButton("选择视频")
        self.select_video_btn.clicked.connect(self.select_video)
        
        self.file_label = QLabel("未选择文件")
        
        file_layout.addWidget(self.select_image_btn)
        file_layout.addWidget(self.select_video_btn)
        file_layout.addWidget(self.file_label)
        file_layout.addStretch()
        file_group.setLayout(file_layout)
        
        # 显示区域（分割窗口）
        splitter = QSplitter(Qt.Horizontal)
        
        # 原始内容显示
        original_group = QGroupBox("原始内容")
        original_layout = QVBoxLayout()
        
        self.original_label = QLabel()
        self.original_label.setAlignment(Qt.AlignCenter)
        self.original_label.setMinimumSize(400, 300)
        self.original_label.setStyleSheet("border: 1px solid gray;")
        self.original_label.setText("请选择文件")
        
        original_layout.addWidget(self.original_label)
        original_group.setLayout(original_layout)
        
        # 识别结果显示
        result_group = QGroupBox("识别结果")
        result_layout = QVBoxLayout()
        
        self.result_label = QLabel()
        self.result_label.setAlignment(Qt.AlignCenter)
        self.result_label.setMinimumSize(400, 300)
        self.result_label.setStyleSheet("border: 1px solid gray;")
        self.result_label.setText("识别结果将显示在这里")
        
        result_layout.addWidget(self.result_label)
        result_group.setLayout(result_layout)
        
        splitter.addWidget(original_group)
        splitter.addWidget(result_group)
        splitter.setSizes([1, 1])
        
        # 控制区域
        control_group = QGroupBox("控制面板")
        control_layout = QHBoxLayout()
        
        self.recognize_btn = QPushButton("开始识别")
        self.recognize_btn.clicked.connect(self.start_recognition)
        self.recognize_btn.setEnabled(False)
        
        self.stop_btn = QPushButton("停止处理")
        self.stop_btn.clicked.connect(self.stop_processing)
        self.stop_btn.setEnabled(False)
        
        control_layout.addWidget(self.recognize_btn)
        control_layout.addWidget(self.stop_btn)
        control_layout.addStretch()
        control_group.setLayout(control_layout)
        
        # 详细结果显示区域
        detail_group = QGroupBox("详细信息")
        detail_layout = QVBoxLayout()
        
        self.detail_text = QTextEdit()
        self.detail_text.setMaximumHeight(120)
        self.detail_text.setReadOnly(True)
        
        detail_layout.addWidget(self.detail_text)
        detail_group.setLayout(detail_layout)
        
        # 添加到主布局
        layout.addWidget(file_group)
        layout.addWidget(splitter)
        layout.addWidget(control_group)
        layout.addWidget(detail_group)
        
        self.setLayout(layout)
    
    def select_image(self):
        """选择图片文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择图片", "", "图片文件 (*.jpg *.jpeg *.png)"
        )
        
        if file_path:
            self.current_file_path = file_path
            self.file_label.setText(f"图片: {os.path.basename(file_path)}")
            
            # 显示图片预览
            pixmap = QPixmap(file_path)
            scaled_pixmap = pixmap.scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.original_label.setPixmap(scaled_pixmap)
            
            # 清空结果显示
            self.result_label.setText("识别结果将显示在这里")
            self.detail_text.clear()
            
            self.recognize_btn.setEnabled(True)
            self.recognize_btn.setText("开始识别")
    
    def select_video(self):
        """选择视频文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择视频", "", "视频文件 (*.mp4 *.avi *.mov)"
        )
        
        if file_path:
            self.current_file_path = file_path
            self.file_label.setText(f"视频: {os.path.basename(file_path)}")
            
            # 显示视频第一帧
            cap = cv2.VideoCapture(file_path)
            ret, frame = cap.read()
            if ret:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                height, width, channel = frame_rgb.shape
                bytes_per_line = 3 * width
                q_image = QPixmap.fromImage(
                    QPixmap.fromImage(
                        QPixmap.fromImage(
                            QPixmap.fromImage(QPixmap()).fromImage(
                                QPixmap().fromImage(QPixmap()).toImage()
                            ).toImage()
                        ).toImage()
                    ).toImage()
                )
                
                # 简化处理：直接转换为PIL然后保存临时文件显示
                temp_path = tempfile.mktemp(suffix='.jpg')
                Image.fromarray(frame_rgb).save(temp_path)
                pixmap = QPixmap(temp_path)
                scaled_pixmap = pixmap.scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.original_label.setPixmap(scaled_pixmap)
                os.remove(temp_path)
            cap.release()
            
            # 清空结果显示
            self.result_label.setText("识别结果将显示在这里")
            self.detail_text.clear()
            
            self.recognize_btn.setEnabled(True)
            self.recognize_btn.setText("开始视频识别")
    
    def start_recognition(self):
        """开始识别"""
        if not self.current_file_path:
            return
        
        # 判断是图片还是视频
        file_ext = os.path.splitext(self.current_file_path)[1].lower()
        
        if file_ext in ['.jpg', '.jpeg', '.png']:
            self.start_image_recognition()
        elif file_ext in ['.mp4', '.avi', '.mov']:
            self.start_video_recognition()
    
    def start_image_recognition(self):
        """开始图片识别"""
        self.recognize_btn.setEnabled(False)
        self.detail_text.setText("识别中...")
        
        # 创建工作线程
        self.worker = RecognitionWorker(self.current_file_path, self.api_key)
        self.worker.finished.connect(self.on_image_recognition_finished)
        self.worker.error.connect(self.on_recognition_error)
        self.worker.start()
    
    def start_video_recognition(self):
        """开始视频识别"""
        self.recognize_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.detail_text.setText("视频识别中...")
        
        # 创建视频工作线程
        self.video_worker = VideoWorker(self.current_file_path, self.api_key)
        self.video_worker.frame_processed.connect(self.on_video_frame_processed)
        self.video_worker.finished.connect(self.on_video_finished)
        self.video_worker.error.connect(self.on_recognition_error)
        self.video_worker.start()
    
    def stop_processing(self):
        """停止处理"""
        if self.video_worker and self.video_worker.isRunning():
            self.video_worker.stop()
            self.video_worker.wait()
        
        self.recognize_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.detail_text.setText("处理已停止")
    
    def on_image_recognition_finished(self, result):
        """图片识别完成回调"""
        plate_number = result.get('plate_number', '未识别')
        
        if plate_number and plate_number != '未识别':
            # 在结果区域显示识别到的车牌号
            self.result_label.setText(f"识别结果: {plate_number}")
            self.result_label.setStyleSheet("""
                border: 1px solid gray;
                background-color: #e8f5e8;
                font-size: 24px;
                font-weight: bold;
                color: #2e7d32;
                padding: 20px;
            """)
            
            # 显示详细信息
            self.detail_text.setText(f"车牌号码: {plate_number}")
        else:
            self.result_label.setText("未能识别出车牌")
            self.result_label.setStyleSheet("""
                border: 1px solid gray;
                background-color: #ffeaea;
                font-size: 18px;
                color: #d32f2f;
                padding: 20px;
            """)
            self.detail_text.setText("识别失败，请尝试更清晰的图片")
        
        self.recognize_btn.setEnabled(True)
    
    def on_video_frame_processed(self, frame, plate_number):
        """视频帧处理回调"""
        # 将numpy数组转换为QPixmap显示
        height, width, channel = frame.shape
        
        # 保存帧到临时文件然后显示
        temp_path = tempfile.mktemp(suffix='.jpg')
        Image.fromarray(frame).save(temp_path)
        pixmap = QPixmap(temp_path)
        scaled_pixmap = pixmap.scaled(400, 300, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.result_label.setPixmap(scaled_pixmap)
        os.remove(temp_path)
        
        # 更新详细信息
        if plate_number:
            self.detail_text.setText(f"当前识别: {plate_number}")
        else:
            self.detail_text.setText("当前帧未识别到车牌")
    
    def on_video_finished(self):
        """视频处理完成回调"""
        self.recognize_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.detail_text.setText("视频处理完成")
    
    def on_recognition_error(self, error_msg):
        """识别错误回调"""
        self.detail_text.setText(f"识别失败: {error_msg}")
        self.recognize_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

class EvaluationTab(QWidget):
    """模型评估功能页面"""
    
    def __init__(self, api_key):
        super().__init__()
        self.api_key = api_key
        self.evaluation_results = []
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # 参数设置区域
        param_group = QGroupBox("评估参数")
        param_layout = QGridLayout()
        
        param_layout.addWidget(QLabel("测试文件夹:"), 0, 0)
        self.folder_label = QLabel("未选择文件夹")
        self.folder_btn = QPushButton("选择文件夹")
        self.folder_btn.clicked.connect(self.select_folder)
        param_layout.addWidget(self.folder_label, 0, 1)
        param_layout.addWidget(self.folder_btn, 0, 2)
        
        param_layout.addWidget(QLabel("测试样本数:"), 1, 0)
        self.sample_spinbox = QSpinBox()
        self.sample_spinbox.setRange(10, 1000)
        self.sample_spinbox.setValue(500)
        param_layout.addWidget(self.sample_spinbox, 1, 1)
        
        param_group.setLayout(param_layout)
        
        # 控制按钮区域
        control_layout = QHBoxLayout()
        self.start_eval_btn = QPushButton("开始评估")
        self.start_eval_btn.clicked.connect(self.start_evaluation)
        self.start_eval_btn.setEnabled(False)
        
        self.save_results_btn = QPushButton("保存结果")
        self.save_results_btn.clicked.connect(self.save_results)
        self.save_results_btn.setEnabled(False)
        
        control_layout.addWidget(self.start_eval_btn)
        control_layout.addWidget(self.save_results_btn)
        control_layout.addStretch()
        
        # 进度显示区域
        progress_group = QGroupBox("评估进度")
        progress_layout = QVBoxLayout()
        
        self.progress_bar = QProgressBar()
        self.progress_label = QLabel("准备就绪")
        
        progress_layout.addWidget(self.progress_bar)
        progress_layout.addWidget(self.progress_label)
        progress_group.setLayout(progress_layout)
        
        # 结果显示区域
        self.result_tabs = QTabWidget()
        
        # 统计信息页面
        self.stats_widget = self.create_stats_widget()
        self.result_tabs.addTab(self.stats_widget, "统计信息")
        
        # 可视化图表页面
        self.chart_widget = self.create_chart_widget()
        self.result_tabs.addTab(self.chart_widget, "可视化图表")
        
        # 错误分析页面
        self.error_widget = self.create_error_widget()
        self.result_tabs.addTab(self.error_widget, "错误分析")
        
        # 添加到主布局
        layout.addWidget(param_group)
        layout.addLayout(control_layout)
        layout.addWidget(progress_group)
        layout.addWidget(self.result_tabs)
        
        self.setLayout(layout)
    
    def create_stats_widget(self):
        """创建统计信息页面"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        self.stats_text = QTextEdit()
        self.stats_text.setReadOnly(True)
        layout.addWidget(self.stats_text)
        
        widget.setLayout(layout)
        return widget
    
    def create_chart_widget(self):
        """创建图表页面"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        # 创建matplotlib画布
        self.figure = Figure(figsize=(12, 8))
        self.canvas = FigureCanvas(self.figure)
        
        layout.addWidget(self.canvas)
        widget.setLayout(layout)
        return widget
    
    def create_error_widget(self):
        """创建错误分析页面"""
        widget = QWidget()
        layout = QVBoxLayout()
        
        self.error_table = QTableWidget()
        self.error_table.setColumnCount(4)
        self.error_table.setHorizontalHeaderLabels(["文件名", "真实车牌", "识别结果", "准确率"])
        
        layout.addWidget(self.error_table)
        widget.setLayout(layout)
        return widget
    
    def select_folder(self):
        """选择测试文件夹"""
        folder_path = QFileDialog.getExistingDirectory(self, "选择测试文件夹")
        
        if folder_path:
            self.test_folder = folder_path
            self.folder_label.setText(os.path.basename(folder_path))
            self.start_eval_btn.setEnabled(True)
    
    def start_evaluation(self):
        """开始评估"""
        if not hasattr(self, 'test_folder'):
            QMessageBox.warning(self, "警告", "请先选择测试文件夹")
            return
        
        sample_size = self.sample_spinbox.value()
        
        self.start_eval_btn.setEnabled(False)
        self.progress_bar.setValue(0)
        
        # 创建评估工作线程
        self.eval_worker = EvaluationWorker(self.test_folder, sample_size, self.api_key)
        self.eval_worker.progress.connect(self.update_progress)
        self.eval_worker.finished.connect(self.on_evaluation_finished)
        self.eval_worker.error.connect(self.on_evaluation_error)
        self.eval_worker.start()
    
    def update_progress(self, current, total, filename):
        """更新进度"""
        progress = int(current / total * 100)
        self.progress_bar.setValue(progress)
        self.progress_label.setText(f"处理第 {current}/{total} 张图片: {os.path.basename(filename)}")
    
    def on_evaluation_finished(self, results):
        """评估完成回调"""
        self.evaluation_results = results
        self.display_results(results)
        self.start_eval_btn.setEnabled(True)
        self.save_results_btn.setEnabled(True)
        self.progress_label.setText("评估完成")
        
        QMessageBox.information(self, "完成", f"评估完成！共处理 {len(results)} 张图片")
    
    def on_evaluation_error(self, error_msg):
        """评估错误回调"""
        QMessageBox.critical(self, "错误", f"评估失败: {error_msg}")
        self.start_eval_btn.setEnabled(True)
        self.progress_label.setText("评估失败")
    
    def display_results(self, results):
        """显示评估结果"""
        if not results:
            return
            
        evaluator = LicensePlateEvaluator(self.api_key)
        stats = evaluator.calculate_statistics(results)
        
        # 显示统计信息
        stats_text = f"""评估统计信息
====================

基本指标:
- 总图片数: {stats.get('total_images', 0)}
- 成功识别数: {stats.get('successful_recognitions', 0)}
- 完全正确数: {stats.get('correct_recognitions', 0)}

性能指标:
- 成功率: {stats.get('success_rate', 0):.1f}%
- 严格匹配率: {stats.get('strict_accuracy', 0):.1f}%
- 字符级准确率: {stats.get('avg_char_accuracy', 0):.1f}%

响应时间:
- 平均响应时间: {stats.get('avg_response_time', 0):.3f} 秒
- 最小响应时间: {stats.get('min_response_time', 0):.3f} 秒
- 最大响应时间: {stats.get('max_response_time', 0):.3f} 秒
- 响应时间标准差: {stats.get('std_response_time', 0):.3f} 秒

错误统计:
- 错误数量: {stats.get('error_count', 0)}"""
        
        self.stats_text.setText(stats_text)
        
        # 绘制图表
        self.plot_charts(results, stats)
        
        # 显示错误分析
        self.display_error_analysis(results)
    
    def plot_charts(self, results, stats):
        """绘制统计图表"""
        self.figure.clear()
        
        # 创建子图
        ax1 = self.figure.add_subplot(2, 2, 1)
        ax2 = self.figure.add_subplot(2, 2, 2)
        ax3 = self.figure.add_subplot(2, 2, 3)
        ax4 = self.figure.add_subplot(2, 2, 4)
        
        # 1. 准确率饼图
        correct_count = stats.get('correct_recognitions', 0)
        incorrect_count = stats.get('total_images', 0) - correct_count
        
        labels = ['完全正确', '有错误']
        sizes = [correct_count, incorrect_count]
        colors = ['lightgreen', 'lightcoral']
        
        ax1.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
        ax1.set_title('识别结果分布')
        
        # 2. 响应时间分布直方图
        response_times = [r['response_time'] for r in results if r['success']]
        if response_times:
            ax2.hist(response_times, bins=20, alpha=0.7, color='skyblue', edgecolor='black')
            ax2.set_xlabel('响应时间 (秒)')
            ax2.set_ylabel('频次')
            ax2.set_title('响应时间分布')
        
        # 3. 准确率分布
        accuracies = [r['strict_accuracy'] for r in results]
        accuracy_counts = [accuracies.count(0), accuracies.count(1)]
        ax3.bar(['错误', '正确'], accuracy_counts, color=['red', 'green'])
        ax3.set_title('准确率分布')
        ax3.set_ylabel('数量')
        
        # 4. 字符级准确率分布
        char_accuracies = [r['char_accuracy'] for r in results]
        ax4.hist(char_accuracies, bins=10, alpha=0.7, color='orange', edgecolor='black')
        ax4.set_xlabel('字符级准确率')
        ax4.set_ylabel('频次')
        ax4.set_title('字符级准确率分布')
        
        self.figure.tight_layout()
        self.canvas.draw()
    
    def display_error_analysis(self, results):
        """显示错误分析"""
        evaluator = LicensePlateEvaluator(self.api_key)
        error_cases = evaluator.get_error_analysis(results, 20)
        
        self.error_table.setRowCount(len(error_cases))
        
        for i, case in enumerate(error_cases):
            self.error_table.setItem(i, 0, QTableWidgetItem(case['filename']))
            self.error_table.setItem(i, 1, QTableWidgetItem(case['actual_plate']))
            self.error_table.setItem(i, 2, QTableWidgetItem(case.get('predicted_plate', 'None')))
            self.error_table.setItem(i, 3, QTableWidgetItem(f"{case['strict_accuracy']*100:.1f}%"))
    
    def save_results(self):
        """保存评估结果"""
        if not self.evaluation_results:
            QMessageBox.warning(self, "警告", "没有评估结果可保存")
            return
        
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存评估结果", f"evaluation_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
            "JSON文件 (*.json)"
        )
        
        if file_path:
            evaluator = LicensePlateEvaluator(self.api_key)
            saved_path = evaluator.save_results(self.evaluation_results, file_path)
            QMessageBox.information(self, "成功", f"结果已保存到: {saved_path}")

class MainWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self):
        super().__init__()
        # 从配置文件加载设置
        Config.load_from_env()
        self.api_key = Config.ARK_API_KEY
        
        # 验证配置
        if not Config.validate_config():
            QMessageBox.critical(self, "配置错误", "请在config.py中配置有效的API密钥")
            sys.exit(1)
        
        self.init_ui()
    
    def init_ui(self):
        self.setWindowTitle(Config.WINDOW_TITLE)
        self.setGeometry(100, 100, *Config.WINDOW_SIZE)
        
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建标签页
        tab_widget = QTabWidget()
        
        # 车牌识别页面
        self.recognition_tab = RecognitionTab(self.api_key)
        tab_widget.addTab(self.recognition_tab, "车牌识别")
        
        # 模型评估页面
        self.evaluation_tab = EvaluationTab(self.api_key)
        tab_widget.addTab(self.evaluation_tab, "模型评估")
        
        # 主布局
        layout = QVBoxLayout()
        
        # 标题
        title_label = QLabel("🚗 车牌识别系统")
        title_label.setAlignment(Qt.AlignCenter)
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        
        layout.addWidget(title_label)
        layout.addWidget(tab_widget)
        
        central_widget.setLayout(layout)
        
        # 设置样式
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #cccccc;
                border-radius: 5px;
                margin-top: 1ex;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QPushButton {
                background-color: #4CAF50;
                border: none;
                color: white;
                padding: 8px 16px;
                text-align: center;
                font-size: 14px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
        """)

def main():
    """主函数"""
    app = QApplication(sys.argv)
    app.setApplicationName("车牌识别系统")
    
    # 设置应用图标（如果有的话）
    # app.setWindowIcon(QIcon('icon.png'))
    
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()