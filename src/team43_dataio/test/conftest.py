"""离线测试直接加载真实纯Python模块；不包含平台兼容实现。"""
from pathlib import Path
import sys

SRC = Path(__file__).resolve().parents[2]
for package in ('team43_dataio', 'team43_common'):
    sys.path.insert(0, str(SRC / package))
