#!/usr/bin/env python3
"""Offline production-path checks: PointCloud2 bytes -> map -> real PGM/YAML.

Synthetic room is explicitly simulated evidence, not NUC accuracy. No ROS,
serial, drivers, hardware or file changes outside a temporary directory.
"""
from __future__ import annotations
import ast
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace as NS
import time
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/grid_mapper'))
from grid_mapper.cloud_grid import CloudGrid, PoseHistory, pointcloud2_xyz, rotation_xyzw
from grid_mapper.grid_builder import FREE, UNKNOWN, OCCUPIED
from grid_mapper.pgm_io import write_map, read_grid, read_pgm


def cloud(points,stamp=100.,frame='camera_init',endian='<',rows=1,padding=12):
    points=np.asarray(points,dtype=float).reshape(-1,3)
    assert len(points)%rows==0
    w=len(points)//rows; point_step=32; row_step=w*point_step+padding
    raw=bytearray(row_step*rows)
    for k,p in enumerate(points):
        row,col=divmod(k,w)
        # nonconsecutive, shuffled fields; x/y float32, z float64.
        struct.pack_into(endian+'f',raw,row*row_step+col*point_step+12,float(p[0]))
        struct.pack_into(endian+'f',raw,row*row_step+col*point_step+0,float(p[1]))
        struct.pack_into(endian+'d',raw,row*row_step+col*point_step+20,float(p[2]))
    fields=[NS(name='z',offset=20,datatype=8,count=1),NS(name='x',offset=12,datatype=7,count=1),
            NS(name='intensity',offset=4,datatype=7,count=1),NS(name='y',offset=0,datatype=7,count=1)]
    sec=int(stamp)
    return NS(width=w,height=rows,point_step=point_step,row_step=row_step,fields=fields,
              data=raw,is_bigendian=endian=='>',header=NS(frame_id=frame,stamp=NS(sec=sec,nanosec=int((stamp-sec)*1e9))))


def assert_raises(call):
    try: call()
    except (ValueError,TypeError): return
    raise AssertionError('invalid input accepted')


def parse_checks():
    p=np.array([[1,2,3],[4,5,6],[7,8,9],[np.nan,1,2]])
    for endian in ('<','>'):
        assert np.array_equal(pointcloud2_xyz(cloud(p,endian=endian,rows=2)),p[:3])
    msg=cloud(p); msg.row_step=1
    assert_raises(lambda:pointcloud2_xyz(msg))
    msg=cloud(p); msg.data=msg.data[:10]
    assert_raises(lambda:pointcloud2_xyz(msg))
    msg=cloud(p); msg.fields[0].count=2
    assert_raises(lambda:pointcloud2_xyz(msg))
    msg=cloud(p); msg.fields[0].name='other'
    assert_raises(lambda:pointcloud2_xyz(msg))
    msg=cloud(p); msg.fields[0].offset=13
    assert_raises(lambda:pointcloud2_xyz(msg))
    return {'layouts':'little/big endian; mixed float32/64; shuffled offsets; organized row padding',
            'finite_filter':True,'malformed_rejected':5}


def pose_checks():
    cfg=yaml.safe_load((ROOT/'src/gimbal_tracker_bringup/config/point_lio_mid360.yaml').read_text())
    mapping=cfg['/**']['ros__parameters']['mapping']
    offset=mapping['extrinsic_T']
    h=PoseHistory(sensor_origin_in_body=offset)
    # Non-yaw-only rotation checks that offset is transformed with full pose.
    a=.7; q=[math.sin(a/2),0.,0.,math.cos(a/2)]; p=np.array([1.,2.,3.])
    h.add(100.,'camera_init','body',p,q)
    assert np.allclose(h.origin(100.),p+rotation_xyzw(q)@np.array(offset))
    assert h.origin(100.1) is None
    assert_raises(lambda:h.add(100.,'camera_init','body',p,q))
    assert_raises(lambda:h.add(101.,'map','body',p,q))
    assert_raises(lambda:h.add(101.,'camera_init','aft_mapped',p,q))
    assert_raises(lambda:h.add(101.,'camera_init','body',p,[0.,0.,0.,0.]))
    return {'offset_read_from_lio_config':offset,'full_rotation_checked':True,
            'out_of_order_frame_child_invalid_q_rejected':True}


def small_checks():
    m=CloudGrid(width_m=8.,height_m=6.,origin_xy=(-4.,-3.))
    st=m.update([[1,0,.5]],[0,0,.8],100.)
    assert st['state']=='需要确认地面高度' and np.all(m.map.grid==UNKNOWN)
    assert m.last_stamp is None
    m.floor_confirmed=True
    for points in ([], [[float('nan'),0.,.5]], [[1.,0.,99.]]):
        empty=m.update(points,[0,0,.8],99.)
        assert empty['rays']==0 and empty['state'].startswith('无有效回波')
        assert m.accepted_scans==0 and m.last_stamp is None
    # one visible obstacle, ignored floor-gap/ceiling, nonfinite point.
    m.update([[1,0,.5],[0,1,2.5],[0,-1,.1],[np.nan,0,.5]],[0,0,.8],100.)
    assert m.map.at_world([1,0])==OCCUPIED
    assert m.map.at_world([.5,0])==FREE
    assert m.map.at_world([0,1])==UNKNOWN
    assert m.map.at_world([2,0])==UNKNOWN # never cleared behind obstacle
    assert np.array_equal(m.map.grid,m.map.ray_grid)
    previous=m.map.grid.copy()
    for points in ([], [[float('nan'),0.,.5]], [[1.,0.,99.]]):
        m.update(points,[0,0,.8],100.5)
        assert m.accepted_scans==1 and m.last_stamp==100.
        assert np.array_equal(m.map.grid,previous)
    assert_raises(lambda:m.update([[1,0,.5]],[0,0,.8],100.))
    assert_raises(lambda:m.update([[1,0,.5]],[50,0,.8],101.))
    # The moving obstacle has gone; a later wall return observes its old cell.
    for i in range(4):m.update([[3,0,.5]],[0,0,.8],101.+i)
    assert m.map.at_world([1,0])==FREE
    assert m.map.at_world([3,0])==OCCUPIED
    return {'unknown_not_filled':True,'height_filter':True,'no_inflation':True,'observed_ghost_cleared':True,'empty_filtered_clouds_do_not_refresh_map':True}


def ray_room(origin,yaw,with_person=False):
    out=[]
    for a in np.linspace(-math.radians(120),math.radians(120),480):
        d=np.array([math.cos(yaw+a),math.sin(yaw+a)])
        ts=[]
        for axis,wall in [(0,-3.),(0,4.),(1,-2.),(1,3.)]:
            if abs(d[axis])<1e-8:continue
            t=(wall-origin[axis])/d[axis]
            hit=origin[:2]+t*d
            if t>.5 and -3.-1e-8<=hit[0]<=4.+1e-8 and -2.-1e-8<=hit[1]<=3.+1e-8: ts.append(t)
        if not ts:continue
        t=min(ts)
        if with_person:
            center=np.array([.8,.35]); rel=origin[:2]-center
            b=2*rel@d; c=rel@rel-.22**2; disc=b*b-4*c
            if disc>=0:
                person=(-b-math.sqrt(disc))/2
                if .5<person<t:t=person
        xy=origin[:2]+t*d
        out.append([xy[0],xy[1],.6])
    return np.asarray(out)


def room_check(tmp):
    m=CloudGrid(resolution=.05,width_m=10.,height_m=8.,origin_xy=(-4.,-3.),floor_confirmed=True)
    cfg=yaml.safe_load((ROOT/'src/gimbal_tracker_bringup/config/point_lio_mid360.yaml').read_text())
    h=PoseHistory(sensor_origin_in_body=cfg['/**']['ros__parameters']['mapping']['extrinsic_T'])
    t0=time.perf_counter(); count=0
    # 120 scans move/yaw the sensor. Register from local sensor coordinates;
    # then the real byte parser, nearest full pose and mapper do the work.
    for k in range(120):
        theta=2*math.pi*k/120; yaw=theta+.3
        p=np.array([.35+.7*math.cos(theta),.4+.55*math.sin(theta),.8])
        q=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)]
        h.add(100.+k*.1,'camera_init','body',p,q)
        origin=h.origin(100.+k*.1)
        world=ray_room(origin,yaw,with_person=k<30)
        rot=rotation_xyzw(q)
        local=(world-origin)@rot
        registered=local@rot.T+origin
        parsed=pointcloud2_xyz(cloud(registered,stamp=100.+k*.1))
        m.update(parsed,origin,100.+k*.1); count+=len(parsed)
    elapsed=time.perf_counter()-t0
    g=m.map
    wall=[]
    for axis,a,b in [(0,-3.,(-2.,3.)),(0,4.,(-2.,3.)),(1,-2.,(-3.,4.)),(1,3.,(-3.,4.))]:
        for v in np.arange(b[0]+.1,b[1]-.05,.05):
            xy=[a,v] if axis==0 else [v,a]
            i,j=g.world_to_cell(xy)
            wall.append(np.any(g.grid[max(0,i-1):i+2,max(0,j-1):j+2]==OCCUPIED))
    interior=[]
    for x in np.arange(-2.8,3.8,.1):
        for y in np.arange(-1.8,2.8,.1):interior.append(g.at_world([x,y])==FREE)
    wall_coverage=float(np.mean(wall)); free_coverage=float(np.mean(interior))
    assert wall_coverage>.97,(wall_coverage,free_coverage)
    assert free_coverage>.97,(wall_coverage,free_coverage)
    assert g.at_world([-.5,-2.5])==UNKNOWN
    ghost=[]
    for x in np.arange(.6,1.01,.025):
        for y in np.arange(.15,.56,.025):ghost.append(g.at_world([x,y])==OCCUPIED)
    ghost_cells=int(np.count_nonzero(ghost)); assert ghost_cells==0,ghost_cells
    paths=write_map(str(tmp),g,'47_slam_map.pgm','47_slam_map.yaml')
    back=read_grid(paths['pgm']); assert np.array_equal(back.grid,g.grid)
    img,_=read_pgm(paths['pgm'])
    # Independent pixel lookups enforce actual image top=y-max, x unflipped.
    for xy in [(0.,2.99),(0.,-1.99),(-2.99,0.),(3.99,0.),(.1,.2),(-.5,-2.5)]:
        i,j=g.world_to_cell(xy); value=g.at((i,j))
        expected={UNKNOWN:205,FREE:254,OCCUPIED:0}[value]
        assert int(img[g.height-1-j,i])==expected
    meta=yaml.safe_load(Path(paths['yaml']).read_text())
    assert meta['origin']==[-4.,-3.,0.] and meta['resolution']==.05
    assert np.array_equal(g.grid,g.ray_grid)
    return {'scans':120,'points':count,'wall_coverage_within_one_cell':wall_coverage,
            'interior_free_coverage':free_coverage,'remaining_observed_person_ghost_samples':ghost_cells,
            'outside_room_unknown':True,'pgm_roundtrip_and_orientation':True,
            'duration_seconds':round(elapsed,4),'scan_hz_this_mac':round(120/elapsed,1)}


def ros_adapter_check(tmp):
    """Execute the real ROS adapter class with only ROS transport substituted."""
    class FakePub:
        def __init__(self):self.messages=[]
        def publish(self,msg):self.messages.append(msg)
    class Msg:
        def __init__(self):
            self.header=NS(frame_id='',stamp=NS(sec=0,nanosec=0))
            self.info=NS(origin=NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=0.)))
    class FakeNode:
        def __init__(self,_):self.parameters={}; self.pubs={}; self.ros_time=100.0
        def declare_parameter(self,k,v):self.parameters[k]=v
        def get_parameter(self,k):return NS(value=self.parameters[k])
        def create_publisher(self,_type,name,*args):p=FakePub();self.pubs[name]=p;return p
        def create_subscription(self,*a):return None
        def create_service(self,*a):return None
        def create_timer(self,*a):return None
        def get_clock(self):return NS(now=lambda:NS(nanoseconds=int(self.ros_time*1e9)))
        def get_name(self):return 'cloud_mapping'
    mod=ast.parse((ROOT/'src/grid_mapper/grid_mapper/cloud_mapping_node.py').read_text())
    nodes=[n for n in mod.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in ['stamp_seconds','CloudMappingNode']]
    from collections import deque
    import os,re
    scope={'Node':FakeNode,
           'latched_qos':lambda depth=1:NS(depth=depth),
           'sensor_qos':lambda depth=10:NS(depth=depth),
           'CloudGrid':CloudGrid,'PoseHistory':PoseHistory,'pointcloud2_xyz':pointcloud2_xyz,
           'OccupancyGrid':Msg,'String':Msg,'PointCloud2':Msg,'Odometry':Msg,'Trigger':Msg,
           'deque':deque,'json':json,'os':os,'re':re,'math':math,'tempfile':tempfile,'time':time,'write_map':write_map,
           'get_logger':lambda node,name:NS(info=lambda *_:None,warning=lambda *_:None)}
    exec(compile(ast.Module(body=nodes,type_ignores=[]),'cloud_mapping_node.py','exec'),scope)
    n=scope['CloudMappingNode']()
    n.origin_model_confirmed=True;n.parameters['floor_confirmed']=True;n.parameters['floor_z']=-.78;n.output=str(tmp/'adapter')
    # Actual Odometry adapter fields, no sensor origin approximations.
    p=NS(x=0.,y=0.,z=.8);q=NS(x=0.,y=0.,z=0.,w=1.)
    odom=NS(header=NS(frame_id='camera_init',stamp=NS(sec=100,nanosec=0)),child_frame_id='body',pose=NS(pose=NS(position=p,orientation=q)))
    n.on_cloud(cloud([[2.,0.,.6]])); assert not n.mapper.accepted_scans
    n.on_odom(odom); assert n.mapper.accepted_scans==1
    n.publish(); saved=n.save()
    assert Path(saved,'47_slam_map.pgm').is_file()
    map_msg=n.pubs['/team47/slam_map'].messages[-1]
    assert map_msg.header.frame_id=='camera_init' and len(map_msg.data)==400*400
    assert map_msg.info.origin.orientation.w==1.
    assert map_msg.info.origin.position.z==-.78
    saved_meta=yaml.safe_load(Path(saved,'47_slam_map.yaml').read_text())
    assert saved_meta['frame_id']=='camera_init'
    assert saved_meta['floor_z']==-.78
    assert saved_meta['origin']==[-10.,-10.,0.] # standard origin[2] remains yaw
    # Real adapter must not save an initial no-evidence map or renew an old one.
    empty=scope['CloudMappingNode']();empty.origin_model_confirmed=True
    empty.parameters['floor_confirmed']=True
    empty.on_odom(odom)
    empty.on_cloud(cloud([],stamp=100.))
    empty.on_cloud(cloud([[1.,0.,99.]],stamp=100.01))
    assert empty.mapper.accepted_scans==0 and empty.mapper.last_stamp is None
    assert_raises(empty.save)
    n.ros_time=100.02
    n.on_cloud(cloud([],stamp=100.01))
    n.on_cloud(cloud([[1.,0.,99.]],stamp=100.02))
    assert n.mapper.accepted_scans==1 and n.mapper.last_stamp==100.
    assert n.last_status['state'].startswith('无有效回波')
    n.publish()
    last_map=n.pubs['/team47/slam_map'].messages[-1]
    assert last_map.header.stamp.sec==100 and last_map.header.stamp.nanosec==0
    assert json.loads(n.pubs['/team47/cloud_mapping/status'].messages[-1].data)['state'].startswith('无有效回波')
    rejected=n.rejected
    n.on_cloud(cloud([[2.,0.,.6]],stamp=100.,frame='map'))
    n.on_cloud(cloud([[2.,0.,.6]],stamp=98.))
    n.on_cloud(cloud([[2.,0.,.6]],stamp=101.))
    n.on_cloud(cloud([[2.,0.,.6]],stamp=100.))
    assert n.rejected==rejected+4
    # Refuse changing floor mid-run rather than mixing two incompatible maps.
    n.parameters['floor_z']=.1;n.process_pending();assert n.rejected==rejected+5
    return {'actual_adapter_class_executed':True,'cloud_before_pose_pairing':True,
            'frame_stale_future_order_rejected':True,'saved_frame_camera_init':True,'nonzero_floor_z_ros_and_yaml_preserved':True,
            'empty_filtered_clouds_neither_save_nor_refresh':True}



def launch_check(tmp):
    tree=ast.parse((ROOT/'src/gimbal_tracker_bringup/launch/cloud_mapping.launch.py').read_text())
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='_setup')
    class LC:
        def __init__(self,key):self.key=key
        def perform(self,context):return context[self.key]
    scope={'yaml':yaml,'LaunchConfiguration':LC,'ParameterValue':lambda value,**kw:value,
           'Node':lambda **kw:kw}
    exec(compile(ast.Module(body=[function],type_ignores=[]),'cloud_mapping.launch.py','exec'),scope)
    original=ROOT/'src/gimbal_tracker_bringup/config/point_lio_mid360.yaml'
    context={'lio_params_file':str(original)}
    output=scope['_setup'](context)
    assert len(output)==1 and output[0]['package']=='grid_mapper'
    actual=output[0]['parameters'][1]['sensor_origin_in_body']
    cfg=yaml.safe_load(original.read_text())
    assert actual==cfg['/**']['ros__parameters']['mapping']['extrinsic_T']
    cfg['/**']['ros__parameters']['mapping']['extrinsic_T']=[.12,-.05,.03]
    modified=tmp/'modified-lio.yaml';modified.write_text(yaml.safe_dump(cfg))
    assert scope['_setup']({'lio_params_file':str(modified)})[0]['parameters'][1]['sensor_origin_in_body']==[.12,-.05,.03]
    cfg['/**']['ros__parameters']['mapping']['extrinsic_est_en']=True
    modified.write_text(yaml.safe_dump(cfg))
    assert_raises(lambda:scope['_setup']({'lio_params_file':str(modified)}))
    return {'lio_extrinsic_loaded_not_duplicated':True,'changed_offset_propagates':True,
            'estimated_extrinsic_rejected':True,'only_one_mapper_no_drivers_tf_motor':True}


def main():
    with tempfile.TemporaryDirectory(prefix='team47-cloud-test-') as td:
        tmp=Path(td)
        result={'pointcloud2':parse_checks(),'pose':pose_checks(),'cell_semantics':small_checks(),
                'synthetic_room':room_check(tmp/'room'),'ros_adapter':ros_adapter_check(tmp),
                'launch':launch_check(tmp)}
    result['scope']='Offline synthetic geometry and real node methods only; no ROS middleware, real bag or NUC accuracy claim.'
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))


if __name__=='__main__':main()
