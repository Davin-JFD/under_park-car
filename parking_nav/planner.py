from __future__ import annotations
import heapq
import math
from typing import Dict, List, Tuple, Optional
from .models import ParkingMap, Point

def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0]-b[0], a[1]-b[1])

def interpolate(points: List[Point], spacing=1.0) -> List[Point]:
    out = [points[0]]
    for a, b in zip(points[:-1], points[1:]):
        n = max(1, math.ceil(dist(a, b) / spacing))
        out.extend((a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n) for i in range(1, n+1))
    return out

class LanePlanner:
    def __init__(self, parking_map: ParkingMap, spacing=1.0, connect_tolerance=2.5, access_distance=12.0):
        self.map = parking_map
        self.spacing = spacing
        self.connect_tolerance = connect_tolerance
        self.access_distance = access_distance
        self.points: List[Point] = []
        self.edges: Dict[int, List[Tuple[int, float]]] = {}
        self.lane_nodes: List[List[int]] = []
        self._build()

    def _add_edge(self, a, b):
        self.edges[a].append((b, dist(self.points[a], self.points[b])))

    def _build(self):
        for lane in self.map.lanes:
            if len(lane.points) < 2:
                continue
            nodes = []
            for p in interpolate(lane.points, self.spacing):
                nodes.append(len(self.points)); self.points.append(p); self.edges[len(self.points)-1] = []
            for a, b in zip(nodes[:-1], nodes[1:]):
                self._add_edge(a, b)
                if lane.bidirectional:
                    self._add_edge(b, a)
            self.lane_nodes.append(nodes)
        for i, nodes_i in enumerate(self.lane_nodes):
            for j, nodes_j in enumerate(self.lane_nodes):
                if i == j: continue
                a, b = nodes_i[-1], nodes_j[0]
                if dist(self.points[a], self.points[b]) <= self.connect_tolerance:
                    self._add_edge(a, b)

    def _nearest_nodes(self, p: Point, incoming: bool, count=8):
        candidates = sorted(((dist(p, q), i) for i, q in enumerate(self.points)))
        usable = []
        for d, i in candidates:
            if d > self.access_distance: break
            if incoming or self.edges.get(i):
                usable.append((d, i))
            if len(usable) >= count: break
        return usable

    def plan(self, start_id: str, end_id: str) -> Tuple[List[Point], float]:
        named = self.map.named_positions()
        if start_id not in named: raise ValueError(f"不存在起点编号: {start_id}")
        if end_id not in named: raise ValueError(f"不存在终点编号: {end_id}")
        return self.plan_points(named[start_id], named[end_id])

    def plan_points(self, start: Point, goal: Point) -> Tuple[List[Point], float]:
        if not self.points: raise ValueError("地图中没有可规划车道")
        starts = self._nearest_nodes(start, False)
        goals = self._nearest_nodes(goal, True)
        if not starts or not goals: raise ValueError("起点或终点距车道太远")
        goal_cost = {n: d for d, n in goals}
        queue, score, parent = [], {}, {}
        for d, n in starts:
            score[n] = d; heapq.heappush(queue, (d + dist(self.points[n], goal), n))
        target = None
        while queue:
            _, u = heapq.heappop(queue)
            if u in goal_cost: target = u; break
            for v, cost in self.edges[u]:
                ng = score[u] + cost
                if ng < score.get(v, float("inf")):
                    score[v] = ng; parent[v] = u
                    heapq.heappush(queue, (ng + dist(self.points[v], goal), v))
        if target is None: raise ValueError("有向车道网络不连通，未找到路径")
        nodes = [target]
        while nodes[-1] in parent: nodes.append(parent[nodes[-1]])
        nodes.reverse()
        path = [start] + [self.points[n] for n in nodes] + [goal]
        clean = [path[0]]
        for p in path[1:]:
            if dist(clean[-1], p) > 1e-6: clean.append(p)
        length = sum(dist(a, b) for a, b in zip(clean[:-1], clean[1:]))
        return clean, length

    def plan_to_parking(self, start_id: str, spot_id: str):
        """规划到车位：路径在"车位旁的可行驶车道"处终止（不下车入位），
        并返回车位相对车辆最终行驶方向的左/右侧，供 UI 提示停车。
        返回 (path, length, side, spot)；side ∈ {'left','right'}。"""
        named = self.map.named_positions()
        if start_id not in named: raise ValueError(f"不存在起点编号: {start_id}")
        spot = next((s for s in self.map.parking_spots if s.id == spot_id), None)
        if spot is None: raise ValueError(f"不存在车位编号: {spot_id}")
        # 先按常规规划到车位中心，再截取到其前方最靠近车位的车道点作为停车点
        path, _ = self.plan_points(named[start_id], spot.position)
        if len(path) < 2: raise ValueError("路径过短，无法生成停车点")
        idx = min(range(len(path) - 1), key=lambda i: dist(path[i], spot.position))
        trimmed = path[:idx + 1]
        # 车位相对车辆行驶方向的左/右侧
        if len(trimmed) >= 2:
            h = math.atan2(trimmed[-1][1] - trimmed[-2][1], trimmed[-1][0] - trimmed[-2][0])
        else:
            h = 0.0
        rel = math.atan2(spot.position[1] - trimmed[-1][1], spot.position[0] - trimmed[-1][0]) - h
        rel = (rel + math.pi) % (2 * math.pi) - math.pi
        side = "left" if rel >= 0 else "right"
        length = sum(dist(a, b) for a, b in zip(trimmed[:-1], trimmed[1:]))
        return trimmed, length, side, spot

def project_to_polyline(point: Point, path: List[Point]):
    """返回 (最近距离, 投影点, 沿路径弧长, 投影点处路径切线航向)。"""
    best = (float("inf"), point, 0.0, 0.0)
    accum = 0.0
    for a, b in zip(path[:-1], path[1:]):
        vx, vy = b[0]-a[0], b[1]-a[1]; l2 = vx*vx+vy*vy
        t = max(0.0, min(1.0, ((point[0]-a[0])*vx+(point[1]-a[1])*vy)/l2)) if l2 else 0.0
        q = (a[0]+t*vx, a[1]+t*vy); d = dist(point, q)
        if d < best[0]:
            tangent = math.atan2(vy, vx)
            best = (d, q, accum + t*math.sqrt(l2), tangent)
        accum += math.sqrt(l2)
    return best
