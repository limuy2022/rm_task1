"""跨会话二维地图读取和点云匹配；不将上午lio_world冒充下午lio_world。"""
from pathlib import Path
import math
import cv2
import numpy as np
import yaml
from .pgm_io import read_pgm, pgm_to_grid


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def angle(x):
    return math.atan2(math.sin(x), math.cos(x))


class SavedMap:
    def __init__(self, path):
        self.path = Path(path).expanduser().resolve()
        meta = yaml.safe_load(self.path.read_text())
        if meta.get('mode', 'trinary') != 'trinary':
            raise ValueError('旧地图必须为trinary模式')
        self.resolution = float(meta['resolution'])
        self.origin = np.asarray(meta['origin'], float)
        if (self.origin.shape != (3,) or not np.isfinite(self.origin).all()
                or not math.isfinite(self.resolution) or not .01 <= self.resolution <= .5):
            raise ValueError('地图分辨率/原点无效')
        img, maximum = read_pgm(str(self.path.parent/meta['image']))
        self.grid = pgm_to_grid(img, float(meta.get('occupied_thresh', .65)),
                                float(meta.get('free_thresh', .196)),
                                int(meta.get('negate', 0)), maximum)
        if np.count_nonzero(self.grid == 100) < 20 or np.count_nonzero(self.grid == 0) < 20:
            raise ValueError('地图没有足够占用和空闲观测，不能重定位')
        # grid内部为[x,y]；距离场也保持同一索引顺序。
        self.distance = cv2.distanceTransform((self.grid != 100).astype(np.uint8),
                                             cv2.DIST_L2, cv2.DIST_MASK_PRECISE)*self.resolution

    def to_cells(self, xy):
        return (np.asarray(xy)-self.origin[:2]) @ rotation(self.origin[2])/self.resolution

    def costs(self, points, poses):
        """points相对当前底盘中心；pose为旧图中的中心xy和新世界到旧世界的yaw。"""
        poses = np.atleast_2d(poses)
        out = []
        for begin in range(0, len(poses), 128):
            batch = poses[begin:begin+128]
            a = batch[:, 2]; c = np.cos(a); s = np.sin(a)
            xy = np.empty((len(batch), len(points), 2))
            xy[:, :, 0] = batch[:, 0, None]+c[:, None]*points[:, 0]-s[:, None]*points[:, 1]
            xy[:, :, 1] = batch[:, 1, None]+s[:, None]*points[:, 0]+c[:, None]*points[:, 1]
            ij = np.floor(self.to_cells(xy)).astype(int)
            valid = ((ij[..., 0] >= 0) & (ij[..., 0] < self.grid.shape[0]) &
                     (ij[..., 1] >= 0) & (ij[..., 1] < self.grid.shape[1]))
            # 边界墙的噪声点可能落在图外一两格：采用随越界距离增加的惩罚，
            # 避免一个栅格边缘的跳变把细化卡在错误位置；独立inlier仍要求在图内。
            outside = np.maximum(-ij, 0)+np.maximum(ij-(np.array(self.grid.shape)-1), 0)
            dist = .15+np.linalg.norm(outside,axis=-1)*self.resolution
            dist[valid] = self.distance[ij[..., 0][valid], ij[..., 1][valid]]
            # 对额外障碍适度鲁棒；远离已测障碍的未知区不算匹配成功。
            unknown = np.zeros(valid.shape, bool)
            unknown[valid] = self.grid[ij[..., 0][valid], ij[..., 1][valid]] == -1
            # 障碍背面常为未知，但距离已测墙体一两格的回波仍可匹配该墙。
            # 只有离所有已测障碍都远的未知区才施加未知惩罚。
            unknown &= dist > .15
            dist[unknown] = np.maximum(dist[unknown], .3)
            out.extend(np.minimum(dist, .6).mean(axis=1))
        return np.asarray(out)

    def match(self, points, hint=None):
        points = np.asarray(points, float)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            raise ValueError('匹配点必须是有限二维数组')
        points = np.unique(np.round(points/.12)*.12, axis=0)
        if len(points) < 40: raise ValueError('有效障碍点不足40个')
        points = points[np.linspace(0, len(points)-1, min(180, len(points))).astype(int)]
        if hint is None:
            step = max(1, int(round(.25/self.resolution)))
            cells = np.argwhere(self.grid[::step, ::step] == 0)*step
            xy = (cells+.5)*self.resolution @ rotation(self.origin[2]).T+self.origin[:2]
            yaws = np.arange(-math.pi, math.pi, math.radians(10))
        else:
            hint = np.asarray(hint, float)
            if hint.shape != (3,) or not np.isfinite(hint).all(): raise ValueError('初始位姿提示无效')
            dx, dy = np.meshgrid(np.arange(-1., 1.01, .2), np.arange(-1., 1.01, .2))
            xy = np.column_stack((dx.ravel(), dy.ravel()))+hint[:2]
            yaws = hint[2]+np.arange(-math.pi/6, math.pi/6+.001, math.radians(5))
        candidates = np.concatenate([np.column_stack((xy, np.full(len(xy), yaw))) for yaw in yaws])
        if not len(candidates): raise ValueError('旧地图中没有可搜索的空闲区域')
        costs = self.costs(points, candidates)
        seeds = []
        for index in np.argsort(costs):
            pose = candidates[index]
            if all(np.linalg.norm(pose[:2]-p[:2]) > .5 or abs(angle(pose[2]-p[2])) > .25 for p in seeds):
                seeds.append(pose.copy())
            if len(seeds) == 12: break
        refined = []
        for pose in seeds:
            for d, da in ((.15, .06), (.05, .02), (.015, .006)):
                for _ in range(30):
                    trial = np.array([pose, pose+[d,0,0], pose+[-d,0,0], pose+[0,d,0],
                                      pose+[0,-d,0], pose+[0,0,da], pose+[0,0,-da]])
                    score = self.costs(points, trial); k = int(np.argmin(score))
                    pose = trial[k]
                    if k == 0: break
            refined.append((float(self.costs(points, pose)[0]), pose))
        refined.sort(key=lambda item:item[0])
        score, best = refined[0]
        alternatives = [s for s,p in refined[1:] if np.linalg.norm(p[:2]-best[:2]) > .6
                        or abs(angle(p[2]-best[2])) > .3]
        gap = min(alternatives)-score if alternatives else .6
        # 近墙匹配率独立于截断均值，不让少量贴墙点掩盖大量不匹配。
        cells = np.floor(self.to_cells(points @ rotation(best[2]).T+best[:2])).astype(int)
        valid = ((cells[:,0]>=0)&(cells[:,0]<self.grid.shape[0])&
                 (cells[:,1]>=0)&(cells[:,1]<self.grid.shape[1]))
        hits = np.zeros(len(points), bool)
        hits[valid] = self.distance[cells[valid,0],cells[valid,1]] <= .15
        inliers = float(hits.mean())
        return {'accepted': bool(score < .15 and inliers >= .65 and gap >= .025),
                'pose': best.tolist(), 'mean_distance_m': score, 'inlier_fraction': inliers,
                'alternative_gap_m': gap, 'points': len(points), 'hint_used': hint is not None}
