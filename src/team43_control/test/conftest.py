# -*- coding: utf-8 -*-
"""Test the real pure control/common modules directly from this source tree."""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))          # .../src/team43_control/test
_PKG_ROOT = os.path.dirname(_HERE)                           # .../src/team43_control
_SRC_ROOT = os.path.dirname(_PKG_ROOT)                       # .../src

for _p in (_PKG_ROOT, os.path.join(_SRC_ROOT, 'team43_common')):
    sys.path.insert(0, _p)
