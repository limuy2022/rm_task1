#!/usr/bin/env python3
"""仅核验数学样例与CSV，不导入ROS、不访问硬件、不修改文件。"""
import csv,json,math,sys
from pathlib import Path
root=Path(__file__).resolve().parent
cfg=json.loads((root/'A_测试输入.json').read_text(encoding='utf-8'))
a=cfg['起点'];c=math.cos(a['yaw_rad']);s=math.sin(a['yaw_rad'])
for case in cfg['位置样例']:
 x,y=case['input'];dx=x-a['x'];dy=y-a['y']
 got=(c*dx+s*dy,-s*dx+c*dy)
 assert max(abs(x-y) for x,y in zip(got,case['expected']))<1e-12
for case in cfg['速度样例']:
 x,y=case['input'];got=(c*x+s*y,-s*x+c*y)
 assert max(abs(x-y) for x,y in zip(got,case['expected']))<1e-12
expected=list(csv.reader((root/'A_匀速轨迹预期.csv').open(encoding='utf-8')))
assert len(expected)==1800
for k,row in enumerate(expected):
 assert len(row)==9
 v=list(map(float,row));assert all(map(math.isfinite,v))
 assert abs(v[0]-v[2]-.04)<1e-6 and abs(v[1]-v[3]+.02)<1e-6
 assert row[8]==f"{1792000000+k/10:.6f}"
if len(sys.argv)>1:
 actual=list(csv.reader(Path(sys.argv[1]).open(encoding='utf-8-sig')))
 assert len(actual)==len(expected),f'行数不符: {len(actual)}'
 for k,(a,e) in enumerate(zip(actual,expected)):
  assert len(a)==9,f'第{k}拍列数不符'
  for j in range(8):
   assert math.isfinite(float(a[j])) and abs(float(a[j])-float(e[j]))<=1e-6,f'第{k}拍第{j+1}列不符'
  assert a[8]==e[8],f'第{k}拍时间列不符'
 print('实际CSV与固定时间合成样例一致。')
print('PASS：5个位置样例、2个速度样例、1800拍匀速预测与时间格式。')
print('未核验ROS、实际源码执行、地图文件、提交延迟或硬件精度。')
