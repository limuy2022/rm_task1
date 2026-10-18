"""离线测试直接加载本包与真实common纯模块。"""
from pathlib import Path
import sys

SRC = Path(__file__).resolve().parents[2]
for package in ('grid_mapper', 'gimbal_common'):
    sys.path.insert(0, str(SRC / package))
