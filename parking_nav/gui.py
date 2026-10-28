from __future__ import annotations
import json, time, math
from pathlib import Path
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (QMainWindow,QWidget,QHBoxLayout,QVBoxLayout,QFormLayout,
    QComboBox,QPushButton,QLabel,QFileDialog,QGroupBox,QPlainTextEdit,QMessageBox,QCheckBox)
from .models import ParkingMap
from .planner import LanePlanner
from .fusion import ImuUwbEkf
from .interfaces import UdpReceiver
from .simulator import SensorSimulator
from .canvas import MapCanvas
ROOT=Path(__file__).resolve().parents[1]

SENSOR_STYLE={'ok':'#2e7d32','warn':'#c62828','off':'#9e9e9e'}

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__(); self.setWindowTitle('地下停车场车载导航测试平台（Windows）'); self.resize(1280,800)
        with open(ROOT/'config/settings.json','r',encoding='utf-8') as f:self.cfg=json.load(f)
        self.map=None;self.planner=None;self.path=[];self.ekf=ImuUwbEkf(self.cfg['fusion']);self.udp=[]
        self.side=None; self._arrived=False
        self.last_seen={'imu':0.0,'uwb':0.0}; self.sensor_count={'imu':0,'uwb':0}; self.source={'imu':None,'uwb':None}
        self.canvas=MapCanvas(); self.logbox=QPlainTextEdit();self.logbox.setReadOnly(True);self.logbox.setMaximumBlockCount(500)
        panel=QWidget(); layout=QVBoxLayout(panel); panel.setMaximumWidth(380)
        # 地图
        files=QGroupBox('地图'); fl=QVBoxLayout(files); open_btn=QPushButton('打开导航地图 JSON');open_btn.clicked.connect(self.open_map); fl.addWidget(open_btn)
        # 点到点规划
        plan=QGroupBox('点到点规划'); form=QFormLayout(plan);self.start=QComboBox();self.end=QComboBox();form.addRow('起点',self.start);form.addRow('终点/车位',self.end)
        pbtn=QPushButton('规划路线');pbtn.clicked.connect(self.plan_route);form.addRow(pbtn)
        # 定位与数据
        loc=QGroupBox('定位与数据'); lv=QVBoxLayout(loc)
        self.status=QLabel('未启动');self.status.setWordWrap(True);lv.addWidget(self.status)
        self.sensor_lbl={k:QLabel(f'● {k.upper()} 未启动') for k in ('imu','uwb')}
        for lbl in self.sensor_lbl.values(): lbl.setWordWrap(True); lv.addWidget(lbl)
        self.mode_lbl=QLabel(''); self.mode_lbl.setWordWrap(True); lv.addWidget(self.mode_lbl)
        udp_row=QHBoxLayout()
        for text,which in [('启动UDP IMU','imu'),('启动UDP UWB','uwb'),('停止UDP','stop')]:
            b=QPushButton(text); b.setFixedWidth(112)
            b.clicked.connect(lambda _=False,w=which: self.stop_udp() if w=='stop' else self.start_udp(w))
            udp_row.addWidget(b)
        lv.addLayout(udp_row)
        self.uwb_check=QCheckBox('启用UWB校正（无UWB停车场请取消勾选）')
        self.uwb_check.setChecked(self.ekf.uwb_enabled)
        self.uwb_check.toggled.connect(self.toggle_uwb); lv.addWidget(self.uwb_check)
        sim_row=QHBoxLayout()
        for text,slot in [('启动内置仿真',self.start_sim),('停止仿真',self.stop_sim),('重置定位',self.reset_localization)]:
            b=QPushButton(text); b.setFixedWidth(112); b.clicked.connect(slot); sim_row.addWidget(b)
        lv.addLayout(sim_row)
        for w in [files,plan,loc,QLabel('运行日志'),self.logbox]:layout.addWidget(w)
        central=QWidget();h=QHBoxLayout(central);h.addWidget(panel);h.addWidget(self.canvas,1);self.setCentralWidget(central)
        self.sim=SensorSimulator();self.sim.imu.connect(self.on_imu);self.sim.uwb.connect(self.on_uwb);self.sim.status.connect(self.log);self.sim.arrived.connect(self.on_arrived)
        self.watchdog=QTimer(self); self.watchdog.setInterval(500); self.watchdog.timeout.connect(self._refresh_status); self.watchdog.start()
        default=ROOT/self.cfg['map_file']
        if default.exists():self.load_map(default)
        self._refresh_status()

    def log(self,s):
        self.logbox.appendPlainText(time.strftime('%H:%M:%S ')+str(s)); self.status.setText(str(s)); self.status.setStyleSheet('')

    def _refresh_status(self):
        now=time.time()
        for key,tmo in (('imu',1.0),('uwb',2.0)):
            lbl=self.sensor_lbl[key]; last=self.last_seen[key]
            if last<=0: state,text='off','未启动/无数据'
            elif now-last<=tmo: state,text='ok','已连接'
            else: state,text='warn','超时(无数据)'
            src={'sim':'仿真','udp':'UDP',None:''}.get(self.source[key],'')
            lbl.setText(f'● {key.upper()} {text}{" · "+src if src else ""}  (收 {self.sensor_count[key]} 帧)')
            lbl.setStyleSheet(f'color:{SENSOR_STYLE[state]}; font-weight:600;')
        uwb=self.ekf.uwb_enabled
        mode='IMU+UWB' if uwb else 'IMU-only(无UWB)'
        tc=self.ekf.correction_stats()
        tctxt=f'开 · 已矫正 {tc["count"]} 次' if tc['enabled'] else '关'
        self.mode_lbl.setText(f'融合模式: {mode}   |   过弯矫正: {tctxt}')
        self.mode_lbl.setStyleSheet('color:#1565c0; font-weight:600;')

    def open_map(self):
        p,_=QFileDialog.getOpenFileName(self,'打开地图',str(ROOT/'maps'),'JSON (*.json)')
        if p:self.load_map(Path(p))

    def load_map(self,p):
        try:
            self.map=ParkingMap.load(p);self.planner=LanePlanner(self.map);self.canvas.set_map(self.map);names=self.map.named_positions();self.start.clear();self.end.clear();self.start.addItems(names.keys());self.end.addItems([x.id for x in self.map.parking_spots] or names.keys());self.log(f'地图已加载: {p.name}')
        except Exception as e:QMessageBox.critical(self,'地图错误',str(e))

    def plan_route(self):
        try:
            end_id=self.end.currentText(); spot_ids={s.id for s in self.map.parking_spots}
            if end_id in spot_ids:
                path,L,side,spot=self.planner.plan_to_parking(self.start.currentText(),end_id)
                self.side=side; self._arrived=False
                self.canvas.set_goal(spot.id, path[-1])
                self.log(f'规划成功 长度{L:.1f}m | 终点为车道停车点，车位 {spot.id} 在您的{"左" if side=="left" else "右"}侧')
            else:
                self.side=None; self._arrived=False
                path,L=self.planner.plan(self.start.currentText(),end_id)
                self.canvas.set_goal(None,None)
                self.log(f'规划成功 长度{L:.1f}m 路径点{len(path)}')
            self.path=path; self.canvas.set_path(path); self.ekf.set_path(path); self.reset_localization(); self._refresh_status()
        except Exception as e:QMessageBox.warning(self,'规划失败',str(e))

    def reset_localization(self):
        if not self.path:self.log('请先规划路线');return
        yaw=math.atan2(self.path[1][1]-self.path[0][1],self.path[1][0]-self.path[0][0]);self.ekf.reset(*self.path[0],yaw);self.ekf.set_path(self.path);self.canvas.clear_track();self.canvas.update_pose(self.ekf.pose());self._arrived=False;self._refresh_status();self.log('定位已重置')

    # ---------- UDP：IMU 与 UWB 已拆分为独立按钮 ----------
    def start_udp(self,which):
        u=self.cfg['udp']; typ='imu' if which=='imu' else 'uwb_position'; key='imu' if which=='imu' else 'uwb'
        port=u['imu_port'] if which=='imu' else u['uwb_port']; slot=self.on_imu if which=='imu' else self.on_uwb
        for r in list(self.udp):
            if r.packet_type==typ: r.stop(); self.udp.remove(r)
        r=UdpReceiver(u['host'],port,typ); r.packet.connect(slot); r.status.connect(self.log); r.start(); self.udp.append(r)
        self.source[key]='udp'

    def stop_udp(self):
        for x in self.udp:x.stop()
        self.udp=[]; self.source['imu']=None; self.source['uwb']=None

    # ---------- 内置仿真 ----------
    def start_sim(self):
        if not self.path:self.plan_route()
        if self.path:
            self.reset_localization(); self.sim.side=self.side; self.sim.set_uwb_enabled(self.ekf.uwb_enabled)
            self.source['imu']='sim'; self.source['uwb']='sim' if self.ekf.uwb_enabled else None
            self.sim.start(self.path)

    def stop_sim(self):
        self.sim.stop(); self.source['imu']=None; self.source['uwb']=None

    # ---------- 传感器数据处理 ----------
    def toggle_uwb(self,checked):
        self.ekf.set_uwb_enabled(checked); self.sim.set_uwb_enabled(checked); self._refresh_status()
        self.log('已启用 UWB 校正（IMU+UWB 融合）' if checked else '已停用 UWB 校正（IMU-only，无UWB停车场模式）')

    def on_imu(self,d):
        self.last_seen['imu']=time.time(); self.sensor_count['imu']+=1
        self.canvas.update_pose(self.ekf.predict(d['timestamp'],d['ax'],d['gyro_z']))
        self._check_arrival()

    def on_uwb(self,d):
        self.last_seen['uwb']=time.time(); self.sensor_count['uwb']+=1
        res=self.ekf.update_uwb(d['timestamp'],d['x'],d['y'],d.get('std'),d.get('quality',1))
        self.canvas.update_uwb(d)
        if res is None:
            self.log('收到 UWB 数据，但校正已停用（无UWB停车场模式）')
        else:
            ok,m=res; self.canvas.update_pose(self.ekf.pose())
            if not ok:self.log(f'UWB 异常值被拒绝，Mahalanobis={m:.2f}')

    def _check_arrival(self):
        if self._arrived or not self.path or self.side is None: return
        last=self.path[-1]; p=self.ekf.pose()
        if math.hypot(p['x']-last[0],p['y']-last[1]) < 1.5:
            self.on_arrived({'side':self.side})

    def on_arrived(self,info):
        if self._arrived: return
        self._arrived=True
        side=self.side or info.get('side'); side_txt='左侧' if side=='left' else '右侧'
        self.log(f'导航结束：已到达可行驶道路停车点，车位在您的{side_txt}')
        self.status.setStyleSheet('color:#c62828; font-weight:700;')
        QMessageBox.information(self,'到达提示',f'已到达停车点，请停车。车位在您的{side_txt}。')

    def closeEvent(self,e):self.stop_udp();self.sim.stop();e.accept()
