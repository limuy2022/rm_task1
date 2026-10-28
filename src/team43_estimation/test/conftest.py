"""Use the production numpy-only estimator and common modules."""
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]
for package in ('team43_estimation', 'team43_common'):
    sys.path.insert(0, str(SRC / package))
