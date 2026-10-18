"""Real wall-clock tick/flush baseline, using the production TickCsvWriter.

A separate OS process tails the still-open file. This measures this host's
scheduler/write/flush/read visibility, NOT ROS/DDS delivery or NUC performance.
No /judge topic, ROS node, serial port or hardware is accessed.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import multiprocessing as mp
import re
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/gimbal_judge_io'))
from gimbal_judge_io.tick_csv import TickCsvWriter


def reader(path, count, timeout, done, pipe):
    rows, errors, tail = [], [], b''
    deadline = time.monotonic()+timeout
    try:
        with open(path, 'rb') as source:
            while len(rows)<count and time.monotonic()<deadline:
                data=source.read()
                if data:
                    tail+=data
                    while b'\n' in tail:
                        raw,tail=tail.split(b'\n',1)
                        seen=time.monotonic_ns()
                        fields=raw.decode('ascii').split(',')
                        if len(fields)!=9 or any(re.fullmatch(r'-?\d+\.\d{6}',f) is None for f in fields):
                            errors.append('invalid CSV line '+str(len(rows)+1))
                        elif not all(math.isfinite(float(f)) for f in fields):
                            errors.append('nonfinite CSV data')
                        rows.append((fields[-1],seen,raw.decode('ascii')))
                elif done.is_set():
                    break
                else:
                    time.sleep(.002)
        pipe.send({'rows':rows,'errors':errors,'partial_tail_bytes':len(tail)})
    except Exception as exc:
        pipe.send({'rows':rows,'errors':errors+[repr(exc)],'partial_tail_bytes':len(tail)})
    finally:
        pipe.close()


def stats(values):
    ordered=sorted(values)
    if not ordered:return {'mean':None,'p95':None,'max':None}
    return {'mean':statistics.mean(ordered),'p95':ordered[max(0,math.ceil(.95*len(ordered))-1)],'max':ordered[-1]}


def benchmark(output, seconds=180., hz=10.):
    if not math.isfinite(seconds) or not math.isfinite(hz) or seconds<=0 or hz<=0:
        raise ValueError('positive duration/hz required')
    count=round(seconds*hz)
    if count<1:raise ValueError('no ticks requested')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    csv_path=output/'47.csv'
    writer=TickCsvWriter(str(csv_path));assert writer.open()
    context=mp.get_context('spawn');done=context.Event();receive,send=context.Pipe(duplex=False)
    process=context.Process(target=reader,args=(str(csv_path),count,seconds+15,done,send))
    process.start();send.close()
    time.sleep(.2)  # reader process can import before the first tick
    start=time.monotonic_ns();period=1e9/hz
    dispatch,scheduled,expected,write_ms={},{},{},[]
    result=None
    try:
        for index in range(count):
            due=start+round((index+1)*period)
            remaining=(due-time.monotonic_ns())*1e-9
            if remaining>0:time.sleep(remaining)
            sent=time.monotonic_ns();t_tick=time.time_ns()*1e-9
            key=f'{t_tick:.6f}';t=index/hz
            values=(1.+.4*(t+.1),2.-.2*(t+.1),1.+.4*t,2.-.2*t,.5,-.25,.3,.4,t_tick)
            dispatch[key]=sent;scheduled[key]=due
            expected[key]=','.join(f'{v:.6f}' for v in values)
            assert writer.write_tick(values[:2],values[2:4],values[4:6],values[6:8],values[8])
            write_ms.append((time.monotonic_ns()-sent)*1e-6)
        # Keep the production writer OPEN until the independent reader has seen all rows.
        if not receive.poll(5.):raise RuntimeError('independent reader did not finish')
        read=receive.recv();done.set();process.join(3.)
        seen=read['rows'];errors=read['errors']
        keys=[entry[0] for entry in seen]
        if len(keys)!=len(set(keys)):errors.append('duplicate tick')
        if keys!=list(expected):errors.append('missing or reordered ticks')
        if any(expected.get(key)!=raw for key,_,raw in seen):errors.append('unexpected data/format')
        visible=[(stamp-dispatch[key])*1e-6 for key,stamp,_ in seen if key in dispatch]
        all_in=[(stamp-scheduled[key])*1e-6 for key,stamp,_ in seen if key in scheduled]
        jitter=[(dispatch[key]-scheduled[key])*1e-6 for key in dispatch]
        over80=sum(value>=80. for value in visible)
        passed=(not errors and len(seen)==count and over80==0 and not read['partial_tail_bytes']
                and writer.is_open and writer.n_dropped==0)
        result={'passed':passed,'scope':'actual production CSV writer, wall-clock scheduler and separate OS reader on this host; no ROS or NUC',
                'duration_requested_s':seconds,'elapsed_s':(time.monotonic_ns()-start)*1e-9,'hz':hz,
                'ticks':count,'rows_seen_while_writer_open':len(seen),'missing_ticks':len(set(expected)-set(keys)),
                'writer_dropped':writer.n_dropped,'format_errors':errors,'partial_tail_bytes':read['partial_tail_bytes'],
                'write_and_flush_ms':stats(write_ms),'dispatch_to_reader_visible_ms':stats(visible),
                'scheduled_to_reader_visible_ms':stats(all_in),'dispatch_jitter_ms':stats(jitter),
                'visible_ge_80ms':over80,'visible_ge_60ms':sum(value>=60. for value in visible),
                'csv_sha256':hashlib.sha256(csv_path.read_bytes()).hexdigest(),
                'ROS_nodes_started':0,'hardware_accessed':False,
                'limitations':['not a ROS /test/* transport test','not a measurement of NUC CPU/disk/DDS','baseline load; not a throughput ceiling stress test']}
        (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        (output/'tick_timing.json').write_text(json.dumps({'tick_keys':list(expected),'dispatch_ns':dispatch,'scheduled_ns':scheduled,'reader_rows':seen})+'\n')
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
        if not passed:raise RuntimeError('baseline failed; preserve result.json')
        return result
    finally:
        done.set();writer.close();receive.close()
        process.join(2.)
        if process.is_alive():process.terminate();process.join()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True)
    parser.add_argument('--seconds',type=float,default=180.)
    parser.add_argument('--hz',type=float,default=10.)
    args=parser.parse_args();benchmark(args.output_dir,args.seconds,args.hz)
