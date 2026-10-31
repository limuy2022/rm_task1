#!/usr/bin/env python3
"""用一个持续存活的ROS节点切换比赛目标，避免多次CLI图发现的偶发漏节点。"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters


def call(node, client, request):
    if not client.wait_for_service(timeout_sec=5.):
        raise RuntimeError('参数服务未就绪：'+client.srv_name)
    future=client.call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
    if not future.done() or future.cancelled():
        raise RuntimeError('参数服务请求超时：'+client.srv_name)
    response=future.result()
    if response is None:raise RuntimeError('参数服务没有返回结果：'+client.srv_name)
    return response


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target_id',type=int,choices=range(50),metavar='ID(0..49)')
    args=parser.parse_args(argv)
    rclpy.init(args=[])
    node=Node('team43_set_target')
    try:
        setters={name:node.create_client(SetParameters,name+'/set_parameters')
                 for name in ('/aruco_detector_node','/marker_pnp_node')}
        getters={name:node.create_client(GetParameters,name+'/get_parameters') for name in setters}
        for name,value in (('/marker_pnp_node',-1),('/aruco_detector_node',args.target_id),
                           ('/marker_pnp_node',args.target_id)):
            request=SetParameters.Request()
            request.parameters=[Parameter('perception.target_digit',value=value).to_parameter_msg()]
            response=call(node,setters[name],request)
            if len(response.results)!=1 or not response.results[0].successful:
                reason=response.results[0].reason if response.results else '无响应项'
                raise RuntimeError(name+'目标设置失败：'+reason)
        for name,client in getters.items():
            request=GetParameters.Request();request.names=['perception.target_digit']
            values=call(node,client,request).values
            if (len(values)!=1 or values[0].type!=ParameterType.PARAMETER_INTEGER
                    or values[0].integer_value!=args.target_id):
                raise RuntimeError(name+'目标读回不符，尚未完成切换')
            print(f'{name}: perception.target_digit={args.target_id}',flush=True)
        print(f'目标已设置为ID {args.target_id}；确认画面和跟踪状态后通知裁判启动本轮客户端。',flush=True)
        return 0
    except (RuntimeError,KeyboardInterrupt) as exc:
        print('目标切换未完成：'+str(exc),file=sys.stderr,flush=True)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
