# 安装和使用指南

## 环境要求

- Python 3.8+
- Windows/macOS/Linux

## 安装步骤

### 1. 安装依赖包

如果遇到numpy版本兼容问题，请使用以下命令：

```bash
# 方法1：使用requirements.txt（推荐）
pip install -r requirements.txt

# 方法2：如果遇到版本冲突，逐个安装
pip install PyQt5
pip install opencv-python
pip install Pillow
pip install numpy
pip install pandas
pip install matplotlib
pip install seaborn
pip install requests
pip install streamlit
```

### 2. 配置API密钥

在 `config.py` 文件中修改API密钥：

```python
ARK_API_KEY = "your_actual_api_key_here"
```

### 3. 运行应用

```bash
# 启动PyQt5 GUI版本
python main.py

# 或者运行Streamlit版本
streamlit run recognition.py
```

## 功能说明

### PyQt5 GUI版本特性

1. **车牌识别**
   - 支持图片格式：JPG, JPEG, PNG
   - 支持视频格式：MP4, AVI, MOV
   - 仅显示识别结果，不进行位置标注
   - 实时视频处理

2. **模型评估**
   - 批量测试功能
   - 性能统计分析
   - 可视化图表
   - 错误案例分析
   - 结果导出

### 界面操作

- **左侧**：显示原始图片/视频
- **右侧**：显示识别结果
- **底部**：显示详细信息和状态

## 故障排除

### 常见问题

1. **ImportError: No module named 'PyQt5'**
   ```bash
   pip install PyQt5
   ```

2. **numpy版本冲突**
   ```bash
   pip install numpy --upgrade
   ```

3. **API调用失败**
   - 检查网络连接
   - 确认API密钥正确
   - 检查API额度

4. **图片无法显示**
   - 确认图片格式支持
   - 检查文件路径正确性

### 性能优化建议

- 使用较小尺寸的图片可提高识别速度
- 批量评估时建议样本数不超过500张
- 视频处理时会消耗较多API调用次数

## 技术支持

如有问题，请检查：
1. Python版本是否为3.8+
2. 所有依赖包是否正确安装
3. API密钥是否配置正确
4. 网络连接是否正常