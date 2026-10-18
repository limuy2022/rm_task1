"""Execute the production launch setup with launch/ROS object stand-ins, no nodes."""
import ast
from pathlib import Path
from types import SimpleNamespace
import os

ROOT=Path(__file__).resolve().parents[1]
class Action:
    def __init__(self,**kw): self.__dict__.update(kw)
class Config:
    def __init__(self,name): self.name=name
    def perform(self,context): return context[self.name]

def main():
    tree=ast.parse((ROOT/'src/gimbal_tracker_bringup/launch/visual_tracking.launch.py').read_text())
    tree.body=[n for n in tree.body if not isinstance(n,(ast.Import,ast.ImportFrom))]
    ns=dict(os=os,get_package_share_directory=lambda _:str(ROOT/'src/gimbal_tracker_bringup'),
        LaunchConfiguration=Config,Node=Action,RegisterEventHandler=lambda e:e,
        OnProcessExit=Action,EmitEvent=Action,Shutdown=Action,
        LaunchDescription=lambda x:x,DeclareLaunchArgument=lambda name,**kw:Action(name=name,**kw),
        OpaqueFunction=Action)
    exec(compile(tree,'production_visual_tracking_launch','exec'),ns)
    description=ns['generate_launch_description']()
    defaults={x.name:x.default_value for x in description if hasattr(x,'name')}
    assert defaults['with_bridge']=='false' and defaults['target_id']=='-1'
    context={**defaults,'target_id':'0'}
    nodes=[n for n in ns['_setup'](context) if hasattr(n,'package')]
    assert [n.package for n in nodes]==['gimbal_perception','gimbal_perception','gimbal_control']
    servo=nodes[-1]
    assert servo.namespace=='team47_tracking' and servo.parameters[-1]['motion_enabled'] is False
    assert servo.parameters[0].endswith('/config/visual_tracking.yaml')
    assert nodes[0].parameters[-1]['perception.marker_corners_topic']=='/team47_tracking/marker_corners'
    enabled=ns['_setup']({**context,'with_bridge':'true'})
    bridge=[n for n in enabled if getattr(n,'package',None)=='gimbal_serial_42'][0]
    p=bridge.parameters[0]
    assert p['subscribe_cmd_vel'] is True and p['publish_tf'] is False
    assert p['port']=='/dev/ttyUSB0' and p['imu_axis_sign']==[1,1,1]
    assert p['cmd_vel_topic']=='/team47_tracking/cmd_vel' and p['imu_topic']=='/red_standard_robot1/gimbal'
    nodes=[n for n in enabled if hasattr(n,'package')]
    handlers=[n for n in enabled if hasattr(n,'target_action')]
    assert len(nodes)==len(handlers) and all(h.target_action in nodes for h in handlers)
    observed=ns['_setup']({**context,'with_bridge':'true','with_debug':'true','debug_csv':'/tmp/example.csv'})
    observer=[n for n in observed if getattr(n,'executable',None)=='tracking_debug_node'][0]
    assert observer.parameters[0]['csv_path']=='/tmp/example.csv'
    assert not any(getattr(h,'target_action',None) is observer for h in observed)
    print('PASS production launch default/no serial, private topic wiring, real bridge parameters, shutdown wiring')

if __name__=='__main__': main()
