from __future__ import annotations
import math, random, time
from PySide6.QtCore import QObject, QTimer, Signal
from .planner import dist

class SensorSimulator(QObject):
    imu = Signal(dict); uwb = Signal(dict); status = Signal(str); arrived = Signal(dict)
    def __init__(self):
        super().__init__(); self.timer=QTimer(self); self.timer.setInterval(20); self.timer.timeout.connect(self._tick)
        self.path=[]; self.segment=0; self.along=0.0; self.speed=2.0; self.t=0.0; self.last_yaw=0.0; self.uwb_elapsed=0.0
        self.uwb_enabled=True; self.side=None; self.arrived_emitted=False; self.true_speed=0.0
    def set_uwb_enabled(self, enabled): self.uwb_enabled=bool(enabled)
    def start(self,path):
        if len(path)<2: self.status.emit("请先规划路径"); return
        self.path=list(path); self.segment=0; self.along=0; self.t=time.time(); self.last_yaw=self._yaw(); self.uwb_elapsed=0; self.arrived_emitted=False; self.true_speed=0.0; self.timer.start(); self.status.emit("仿真已启动")
    def stop(self): self.timer.stop(); self.status.emit("仿真已停止")
    def _yaw(self):
        a,b=self.path[self.segment],self.path[min(self.segment+1,len(self.path)-1)]
        return math.atan2(b[1]-a[1],b[0]-a[0])
    def _position(self):
        a,b=self.path[self.segment],self.path[self.segment+1]; L=dist(a,b); r=min(1,self.along/max(L,1e-9))
        return a[0]+r*(b[0]-a[0]),a[1]+r*(b[1]-a[1])
    def _emit_arrived(self):
        if not self.arrived_emitted:
            self.arrived_emitted=True
            self.stop()
            self.arrived.emit({"side": self.side})
    def _tick(self):
        dt=0.02; self.t+=dt; self.uwb_elapsed+=dt
        if self.segment>=len(self.path)-1: self._emit_arrived(); return
        # 先算本帧真实加速度/速度（从0平滑加速到巡航速度），虚拟车辆与EKF用同一速度前进，避免起步滞后
        ax_true=max(-1.0,min(1.0,(self.speed-self.true_speed)*2.0))
        self.true_speed=max(0.0,self.true_speed+ax_true*dt)
        L=dist(self.path[self.segment],self.path[self.segment+1]); self.along+=self.true_speed*dt
        if self.along>=L:
            self.along-=L; self.segment+=1
            if self.segment>=len(self.path)-1: self._emit_arrived(); return
        yaw=self._yaw(); dyaw=(yaw-self.last_yaw+math.pi)%(2*math.pi)-math.pi
        gyro=dyaw/dt + random.gauss(0,0.015) + 0.002; self.last_yaw=yaw
        # 加速度噪声取车规IMU匀速巡航水平(0.03)；过大(如0.08)会使EKF速度随机游走，
        # 在长直道末端累积数米纵向滞后（纯IMU无外部速度参考无法观测，需轮速计/OBD）。
        ax=ax_true+random.gauss(0,0.03)
        self.imu.emit({"type":"imu","timestamp":self.t,"ax":ax,"gyro_z":gyro})
        if self.uwb_enabled and self.uwb_elapsed>=0.2:
            self.uwb_elapsed=0; x,y=self._position()
            self.uwb.emit({"type":"uwb_position","timestamp":self.t,"x":x+random.gauss(0,0.35),"y":y+random.gauss(0,0.35),"std":0.45,"quality":1.0})
