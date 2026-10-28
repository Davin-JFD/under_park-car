from __future__ import annotations
import math
import numpy as np
from .planner import project_to_polyline

def wrap(a): return (a + math.pi) % (2*math.pi) - math.pi


class TurnCorrector:
    """基于"过弯 / 掉头"的累计误差矫正（转弯完成后才矫正，避免过转）。

    规划路径上每个航向变化超过阈值(默认35°)的明显转角，都被记录为一个
    "转角点"（位置 + 转弯后应沿的方向 heading_after + 应转的角度 delta）。

    关键：检测到转弯(累计转角达到阈值)只进入"转弯中"状态，**不立即矫正**；
    只有当车辆确实转过了目标角度(累计转角≈delta)、或已停止转动且转够了
    delta-align 时，才把航向对齐到 heading_after、并把位置吸附到转角点。
    这样矫正发生在出弯瞬间，避免"提前把航向摆到目标方向、随后陀螺角速度
    又被积分一次"导致的出弯过转/欠转与斜向漂移。
    """
    def __init__(self, path=None, threshold_deg=35.0, corner_tolerance=6.0,
                 snap_position=True, cooldown_dist=3.0,
                 align_deg=6.0, stop_rate_deg=1.5):
        self.threshold = math.radians(float(threshold_deg))
        self.corner_tolerance = float(corner_tolerance)
        self.snap_position = bool(snap_position)
        self.cooldown_dist = float(cooldown_dist)
        self.align_deg = math.radians(float(align_deg))     # 判定"已转过目标角度"的容差
        self.stop_step = math.radians(float(stop_rate_deg)) # 每步角速度低于此值视为"停止转动"
        self.turns = []
        self.acc = 0.0                # 自上次矫正以来累计的(带符号)航向变化
        self.last_yaw = None          # 上一次喂入的航向
        self.last_corner_pos = None   # 最近一次矫正的转角点（冷却用）
        self.turning = False          # 是否处于"转弯中"状态
        self.turn_target = None       # 当前正在转的目标转角点
        self.corrections = 0          # 累计矫正次数
        self.last_event = None        # 最近一次矫正信息
        self.set_path(path or [])

    def set_path(self, path):
        self.turns = self._extract(list(path))
        self.reset_tracking()

    def reset_tracking(self):
        self.acc = 0.0
        self.last_yaw = None
        self.last_corner_pos = None
        self.turning = False
        self.turn_target = None

    def _extract(self, path):
        """从规划路径提取明显转角点。"""
        turns = []
        if len(path) < 3:
            return turns
        for i in range(1, len(path) - 1):
            a, b, c = path[i - 1], path[i], path[i + 1]
            h1 = math.atan2(b[1] - a[1], b[0] - a[0])
            h2 = math.atan2(c[1] - b[1], c[0] - b[0])
            delta = wrap(h2 - h1)
            if abs(delta) >= self.threshold:
                turns.append({"pos": tuple(b), "delta": delta, "heading_after": h2})
        return turns

    def _match_corner(self, x, y):
        """在容差内找离当前位置最近的转角点。"""
        best, bd = None, self.corner_tolerance
        for t in self.turns:
            d = math.hypot(x - t["pos"][0], y - t["pos"][1])
            if d < bd:
                bd, best = d, t
        return best

    def on_pose(self, pose):
        """喂入当前位姿；若本次转弯已结束则返回矫正信息 dict，否则返回 None。"""
        x, y, yaw = pose["x"], pose["y"], pose["yaw"]
        if self.last_yaw is None:
            self.last_yaw = yaw
            return None
        dy = wrap(yaw - self.last_yaw)
        self.last_yaw = yaw
        # 冷却期：刚矫正完，车辆还没离开转角点前不再累计/触发
        if self.last_corner_pos is not None and \
           math.hypot(x - self.last_corner_pos[0], y - self.last_corner_pos[1]) < self.cooldown_dist:
            return None

        if not self.turning:
            # 累计转角达到阈值 -> 进入"转弯中"，先不矫正
            self.acc += dy
            if abs(self.acc) >= self.threshold:
                t = self._match_corner(x, y)
                if t is not None:
                    self.turning = True
                    self.turn_target = t
                else:
                    # 不在任何转角附近：多半是慢漂移而非真实转弯，清零避免误触发
                    self.acc = 0.0
            return None

        # 转弯进行中：继续累计，直到"车辆停止转动且已转够目标角度"再矫正
        self.acc += dy
        d = self.turn_target["delta"]
        if abs(dy) < self.stop_step and abs(self.acc) >= abs(d) - self.align_deg:
            return self._apply(self.turn_target, x, y)
        return None

    def _apply(self, t, x, y):
        """出弯瞬间：对齐航向到转弯后方向、吸附位置到转角点并复位累计。"""
        self.last_corner_pos = t["pos"]
        self.acc = 0.0
        self.last_yaw = t["heading_after"]
        self.turning = False
        self.turn_target = None
        self.corrections += 1
        corr = {"pos": t["pos"], "yaw": t["heading_after"], "delta": t["delta"],
                "dist": math.hypot(x - t["pos"][0], y - t["pos"][1]), "index": self.corrections}
        self.last_event = corr
        return corr


class ImuUwbEkf:
    """2D 车辆模型 EKF。状态 [x,y,v,yaw,gyro_bias]。

    两种模式（IMU 与 UWB 已拆分，可独立开关）：
      - IMU+UWB（默认，uwb_enabled=True）：IMU 预测，UWB(x,y) 校正；
      - IMU-only（uwb_enabled=False，用于没有 UWB 的停车场）：
        仅由 IMU 航位推算，配合"过弯矫正"周期性消除累计误差。
    """
    def __init__(self, cfg=None):
        c = cfg or {}
        self.acc_noise = float(c.get("imu_accel_noise", 0.8))
        self.gyro_noise = float(c.get("gyro_noise", 0.04))
        self.bias_noise = float(c.get("gyro_bias_noise", 0.003))
        self.uwb_std = float(c.get("uwb_default_std", 0.6))
        self.gate = float(c.get("uwb_gate_mahalanobis", 9.21))
        self.map_match_max = float(c.get("map_match_max_distance", 4.0))
        self.map_match_weight = float(c.get("map_match_weight", 0.10))
        self.uwb_enabled = bool(c.get("uwb_enabled", True))
        tc = c.get("turn_correction", {}) or {}
        self.turn_correction_enabled = bool(tc.get("enabled", True))
        self.turn_corrector = None
        if self.turn_correction_enabled:
            self.turn_corrector = TurnCorrector(
                path=None,
                threshold_deg=tc.get("threshold_deg", 35.0),
                corner_tolerance=tc.get("corner_tolerance", 6.0),
                snap_position=tc.get("snap_position", True),
                cooldown_dist=tc.get("cooldown_dist", 3.0),
                align_deg=tc.get("align_deg", 6.0),
                stop_rate_deg=tc.get("stop_rate_deg", 1.5),
            )
        self.path = []
        self.reset()

    def reset(self, x=0.0, y=0.0, yaw=0.0):
        self.x = np.array([x, y, 0.0, yaw, 0.0], dtype=float)
        self.P = np.diag([4.0, 4.0, 2.0, 0.5, 0.02])
        self.last_t = None; self.accepted_uwb = 0; self.rejected_uwb = 0
        if self.turn_corrector is not None:
            self.turn_corrector.reset_tracking()

    def set_path(self, path):
        self.path = list(path)
        if self.turn_corrector is not None:
            self.turn_corrector.set_path(self.path)

    def set_uwb_enabled(self, enabled):
        self.uwb_enabled = bool(enabled)

    def predict(self, timestamp, ax, gyro_z):
        t = float(timestamp)
        if self.last_t is None: self.last_t = t; return self.pose()
        dt = max(0.001, min(0.2, t-self.last_t)); self.last_t = t
        px, py, v, yaw, bg = self.x
        omega = gyro_z-bg
        yaw2 = wrap(yaw+omega*dt)
        v2 = v+ax*dt
        self.x = np.array([px+v2*math.cos(yaw2)*dt, py+v2*math.sin(yaw2)*dt, v2, yaw2, bg])
        F = np.eye(5)
        F[0,2]=math.cos(yaw2)*dt; F[1,2]=math.sin(yaw2)*dt
        F[0,3]=-v2*math.sin(yaw2)*dt; F[1,3]=v2*math.cos(yaw2)*dt
        F[3,4]=-dt
        G = np.zeros((5,3)); G[0,0]=0.5*math.cos(yaw2)*dt*dt; G[1,0]=0.5*math.sin(yaw2)*dt*dt
        G[2,0]=dt; G[3,1]=dt; G[4,2]=dt
        Q=np.diag([self.acc_noise**2,self.gyro_noise**2,self.bias_noise**2])
        self.P=F@self.P@F.T+G@Q@G.T
        self._soft_map_match()
        self._apply_turn_correction()
        return self.pose()

    def _apply_turn_correction(self):
        if self.turn_corrector is None:
            return None
        corr = self.turn_corrector.on_pose(self.pose())
        if corr is not None:
            if self.turn_corrector.snap_position:
                self.x[0], self.x[1] = corr["pos"][0], corr["pos"][1]
            self.x[3] = corr["yaw"]
            self._soft_map_match()
        return corr

    def update_uwb(self, timestamp, x, y, std=None, quality=1.0):
        if not self.uwb_enabled:
            return None
        z=np.array([float(x),float(y)]); H=np.zeros((2,5)); H[0,0]=H[1,1]=1.0
        s=max(0.05, float(std or self.uwb_std))/max(0.2,float(quality))
        R=np.eye(2)*s*s; innovation=z-H@self.x; S=H@self.P@H.T+R
        mahal=float(innovation.T@np.linalg.inv(S)@innovation)
        if mahal > self.gate:
            self.rejected_uwb += 1; return False, mahal
        K=self.P@H.T@np.linalg.inv(S); self.x=self.x+K@innovation; self.x[3]=wrap(self.x[3])
        I=np.eye(5); self.P=(I-K@H)@self.P@(I-K@H).T+K@R@K.T
        self.accepted_uwb += 1; self._soft_map_match(); return True, mahal

    def _soft_map_match(self):
        if len(self.path)<2 or self.map_match_weight<=0: return
        d,q,s,tangent=project_to_polyline((self.x[0],self.x[1]),self.path)
        if d>self.map_match_max: return
        # 仅当 EKF 航向与路径切线夹角<20°（车辆真正在直道上行驶）时才软约束；
        # 拐角/转弯处不干预，让陀螺自由积分过弯，否则会把位置拽回上一段路导致卡死。
        dyaw=wrap(tangent-self.x[3])
        if abs(dyaw)>=math.radians(20): return
        w=self.map_match_weight
        # 只修正横向偏差（垂直于路径方向），不纵向拖拽位置，避免车被拉回起点。
        # 纵向累计误差无法靠地图投影消除（投影弧长推进速率本质=EKF自身速度，循环），
        # 纵向误差统一由"过弯矫正"在每个转角点位置吸附清零。
        nx,ny=-math.sin(tangent),math.cos(tangent)
        lat=(q[0]-self.x[0])*nx+(q[1]-self.x[1])*ny
        self.x[0]+=w*lat*nx; self.x[1]+=w*lat*ny
        # 航向软约束：直道上把航向拉向路径切线，抑制陀螺零偏导致的原地旋转。
        self.x[3]=wrap(self.x[3]+0.04*dyaw)

    def pose(self):
        return {"x":float(self.x[0]),"y":float(self.x[1]),"speed":float(self.x[2]),
                "yaw":float(self.x[3]),"gyro_bias":float(self.x[4]),
                "sigma_x":float(math.sqrt(max(0,self.P[0,0]))),"sigma_y":float(math.sqrt(max(0,self.P[1,1])))}

    def correction_stats(self):
        if self.turn_corrector is None:
            return {"enabled": False, "count": 0, "last": None}
        return {"enabled": True, "count": self.turn_corrector.corrections,
                "last": self.turn_corrector.last_event}
