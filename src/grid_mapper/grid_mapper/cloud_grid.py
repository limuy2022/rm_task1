"""Registered PointCloud2 -> observed 2-D occupancy; independent of ROS.

Points and sensor origin must already share one gravity-aligned source frame.
No guessed floor, no conversion of an unobserved cell into free space, no map
inflation. GridMap/PGM conventions are reused from the original package.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from .grid_builder import GridMap, UNKNOWN, FREE, OCCUPIED


_TYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def pointcloud2_xyz(msg, max_points=1000000):
    """Read real PointCloud2 field offsets/endian/point_step/row_step.

    Structured numpy strides also support organized clouds and row padding.
    Only finite xyz are returned. Count != 1 and invalid layouts are rejected.
    """
    w, h, step, row = map(int, (msg.width, msg.height, msg.point_step, msg.row_step))
    if w < 0 or h < 0 or w*h > max_points or step <= 0 or row < w*step:
        raise ValueError('invalid PointCloud2 dimensions/strides')
    endian = '>' if bool(msg.is_bigendian) else '<'
    names, formats, offsets, spans = [], [], [], []
    for name in ('x', 'y', 'z'):
        fields = [f for f in msg.fields if f.name == name]
        if len(fields) != 1:
            raise ValueError('PointCloud2 needs exactly one '+name+' field')
        f = fields[0]
        if int(f.count) != 1 or int(f.datatype) not in _TYPES:
            raise ValueError('invalid xyz datatype/count')
        dt = np.dtype(endian+_TYPES[int(f.datatype)])
        off = int(f.offset)
        if off < 0 or off+dt.itemsize > step:
            raise ValueError('xyz field outside point_step')
        names.append(name); formats.append(dt); offsets.append(off); spans.append((off,off+dt.itemsize))
    if any(max(a,c)<min(b,d) for i,(a,b) in enumerate(spans) for c,d in spans[i+1:]):
        raise ValueError('xyz fields overlap')
    required = (h-1)*row+w*step if h and w else 0
    data = memoryview(msg.data)
    if data.nbytes < required:
        raise ValueError('truncated PointCloud2 payload')
    if not w or not h:
        return np.empty((0, 3), dtype=float)
    dtype = np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': step})
    structured = np.ndarray((h, w), dtype=dtype, buffer=data, strides=(row, step))
    xyz = np.column_stack([structured[n].reshape(-1) for n in names]).astype(float)
    return xyz[np.isfinite(xyz).all(axis=1)]


def rotation_xyzw(q):
    q = np.asarray(q, dtype=float).reshape(4)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 0.5 or norm > 1.5:
        raise ValueError('invalid odometry quaternion')
    x, y, z, w = q/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


@dataclass
class PoseSample:
    stamp: float
    frame: str
    child: str
    position: np.ndarray
    rotation: np.ndarray


class PoseHistory:
    """Bounded matching of cloud stamps to full body poses, never latest-only."""
    def __init__(self, frame='camera_init', child='body', max_delta=0.04,
                 sensor_origin_in_body=None):
        self.frame, self.child, self.max_delta = str(frame), str(child), float(max_delta)
        if sensor_origin_in_body is None:
            raise ValueError('sensor origin must be provided from the same Point-LIO configuration')
        self.offset = np.asarray(sensor_origin_in_body, dtype=float).reshape(3)
        if not np.isfinite(self.offset).all() or not 0 < self.max_delta <= 0.2:
            raise ValueError('invalid sensor origin/time tolerance')
        self.samples = []

    def add(self, stamp, frame, child, position, quaternion_xyzw):
        stamp = float(stamp)
        p = np.asarray(position, dtype=float).reshape(3)
        r = rotation_xyzw(quaternion_xyzw)
        if frame != self.frame or child != self.child:
            raise ValueError('odometry frame/child mismatch')
        if not math.isfinite(stamp) or stamp <= 0 or not np.isfinite(p).all():
            raise ValueError('invalid odometry stamp/position')
        if self.samples and stamp <= self.samples[-1].stamp:
            raise ValueError('out-of-order odometry (restart mapper after replay reset)')
        self.samples.append(PoseSample(stamp, frame, child, p, r))
        self.samples = self.samples[-100:]

    def origin(self, stamp):
        if not self.samples:
            return None
        pose = min(self.samples, key=lambda p: abs(p.stamp-stamp))
        if abs(pose.stamp-stamp) > self.max_delta:
            return None
        return pose.position+pose.rotation@self.offset


class CloudGrid:
    """Evidence grid: hits +3, clear rays -2, bounded [-6, 9].

    Re-observing empty space clears moving-object ghosts. Unobserved ghosts are
    not erased on a timer: a timer cannot prove that a cell is free. Endpoints
    from this scan win over clearing rays from the same scan.
    """
    def __init__(self, resolution=0.05, width_m=20., height_m=20., origin_xy=(-10., -10.),
                 floor_z=0., floor_confirmed=False, obstacle_min=0.15, obstacle_max=1.8,
                 ground_tolerance=0.08, min_range=0.5, max_range=20., angle_bin_deg=0.5):
        values = [resolution, width_m, height_m, floor_z, obstacle_min, obstacle_max,
                  ground_tolerance, min_range, max_range, angle_bin_deg, *origin_xy]
        if not np.isfinite(values).all() or resolution <= 0 or min(width_m,height_m) <= 0:
            raise ValueError('invalid map geometry')
        if not (0 <= ground_tolerance < obstacle_min < obstacle_max and 0 <= min_range < max_range):
            raise ValueError('invalid height/range bands')
        if not (0.1 <= angle_bin_deg <= 5.) or width_m*height_m/resolution**2 > 4000000:
            raise ValueError('map/ray bounds too large')
        self.floor_z, self.floor_confirmed = float(floor_z), bool(floor_confirmed)
        self.obstacle_min, self.obstacle_max = float(obstacle_min), float(obstacle_max)
        self.ground_tolerance = float(ground_tolerance)
        self.min_range, self.max_range = float(min_range), float(max_range)
        self.angle_bin = math.radians(angle_bin_deg)
        shape = (int(math.ceil(width_m/resolution)), int(math.ceil(height_m/resolution)))
        self.map = GridMap(resolution=resolution, origin_xy=origin_xy,
                           grid=np.full(shape, UNKNOWN, dtype=np.int8), inflation_radius_m=0.)
        self.evidence = np.zeros(shape, dtype=np.int16)
        self.observed = np.zeros(shape, dtype=bool)
        self.last_stamp = None
        self.accepted_scans = 0

    def update(self, xyz, sensor_origin, stamp):
        points = np.asarray(xyz, dtype=float).reshape(-1, 3)
        origin = np.asarray(sensor_origin, dtype=float).reshape(3)
        stamp = float(stamp)
        if not math.isfinite(stamp) or stamp <= 0 or not np.isfinite(origin).all():
            raise ValueError('invalid cloud stamp/sensor origin')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            raise ValueError('out-of-order cloud (restart mapper after replay reset)')
        points = points[np.isfinite(points).all(axis=1)]
        stats = {'input_finite': len(points), 'floor_confirmed': self.floor_confirmed}
        stats['z_percentiles'] = np.percentile(points[:,2], [5,50,95]).tolist() if len(points) else []
        if not self.floor_confirmed:
            return {**stats, 'state': '需要确认地面高度', 'rays': 0, 'hits': 0}
        if not self.map.world_in_bounds(origin[:2]):
            raise ValueError('sensor origin outside configured map; enlarge map explicitly')
        rel = points[:,:2]-origin[:2]
        dist = np.linalg.norm(rel, axis=1)
        z = points[:,2]-self.floor_z
        hit = (z >= self.obstacle_min)&(z <= self.obstacle_max)
        ground = np.abs(z) <= self.ground_tolerance
        keep = (hit|ground)&(dist >= self.min_range)&(dist <= self.max_range)
        p, hit, rel, dist = points[keep], hit[keep], rel[keep], dist[keep]
        if not len(p):
            # An empty/fully filtered scan supplies no map evidence. Keep the
            # last valid map timestamp; do not make an empty map savable.
            return {**stats, **self.map.stats(), 'state':'无有效回波，保留上一张地图',
                    'rays':0, 'hits':0, 'accepted_scans':self.accepted_scans}
        occupied = np.zeros(self.map.shape, dtype=bool)
        free = np.zeros(self.map.shape, dtype=bool)
        if len(p):
            ij = np.floor((p[:,:2]-self.map.origin_xy)/self.map.resolution).astype(int)
            in_map = (ij[:,0]>=0)&(ij[:,0]<self.map.width)&(ij[:,1]>=0)&(ij[:,1]<self.map.height)
            for i,j in ij[hit&in_map]:
                occupied[i,j] = True
            # One closest ray per bearing bin prevents a distant floor return
            # from clearing through a nearer visible obstacle in that direction.
            bins = np.floor((np.arctan2(rel[:,1],rel[:,0])+math.pi)/self.angle_bin).astype(int)
            order = np.argsort(dist, kind='stable')
            _, first = np.unique(bins[order], return_index=True)
            chosen = order[first]
            start = self.map.world_to_cell(origin[:2])
            for k in chosen:
                endpoint = tuple(ij[k])
                cells = self._line(start, endpoint)
                for i,j in cells:
                    if not self.map.in_bounds((i,j)):
                        break
                    if (i,j) == endpoint and hit[k]:
                        break
                    free[i,j] = True
            rays = len(chosen)
        else:
            rays = 0
        free &= ~occupied
        self.observed |= free|occupied
        self.evidence[free] = np.maximum(-6, self.evidence[free]-2)
        self.evidence[occupied] = np.minimum(9, self.evidence[occupied]+3)
        self.map.grid[:] = UNKNOWN
        self.map.grid[self.observed&(self.evidence <= -1)] = FREE
        self.map.grid[self.observed&(self.evidence >= 2)] = OCCUPIED
        self.map.ray_grid[:] = self.map.grid
        self.last_stamp = stamp
        self.accepted_scans += 1
        return {**stats, **self.map.stats(), 'state':'正在建图', 'rays':rays,
                'hits': int(occupied.sum()), 'accepted_scans': self.accepted_scans}

    @staticmethod
    def _line(start, end):
        """Integer Bresenham inclusive; clipped by caller at map boundary."""
        x,y = start; ex,ey = end
        dx,dy = abs(ex-x),abs(ey-y)
        sx,sy = (1 if x<ex else -1), (1 if y<ey else -1)
        err = dx-dy
        while True:
            yield x,y
            if x==ex and y==ey: break
            e2=2*err
            if e2 > -dy: err-=dy; x+=sx
            if e2 < dx: err+=dx; y+=sy
