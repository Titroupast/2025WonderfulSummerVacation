#!/usr/bin/env python3
"""
车牌识别系统主启动文件
"""

if __name__ == "__main__":
    try:
        from gui_main import main
        main()
    except ImportError as e:
        print(f"导入错误: {e}")
        print("请安装所需依赖包：pip install -r requirements.txt")
    except Exception as e:
        print(f"启动失败: {e}")