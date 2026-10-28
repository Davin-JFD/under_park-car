from pathlib import Path
import math, random
from parking_nav.models import ParkingMap
from parking_nav.planner import LanePlanner, dist
from parking_nav.fusion import ImuUwbEkf, TurnCorrector
ROOT=Path(__file__).resolve().parents[1]

def test_plan():
    m=ParkingMap.load(ROOT/'maps/demo_map.json'); p,L=LanePlanner(m).plan('入口A','P031'); assert len(p)>2 and L>0

def test_filter():
    f=ImuUwbEkf(); f.reset(0,0,0); f.predict(1,0,0); f.predict(1.1,1,0); ok,_=f.update_uwb(1.1,.01,0,1); assert ok

def test_imu_only():
    # 无 UWB 停车场模式：UWB 校正被忽略，仅 IMU 航位推算
    f=ImuUwbEkf({'uwb_enabled':False}); f.reset(0,0,0)
    assert f.update_uwb(1.0,5,5) is None

def test_plan_to_parking():
    m=ParkingMap.load(ROOT/'maps/demo_map.json'); pl=LanePlanner(m)
    path,L,side,spot=pl.plan_to_parking('入口A','P031')
    assert len(path)>=2 and L>0 and side in ('left','right')
    # 停车点应在可行驶车道上，而非车位中心
    d=math.hypot(path[-1][0]-spot.position[0], path[-1][1]-spot.position[1])
    assert d>0.5

def test_turn_correction():
    # 构造 L 形路径：先直行再 90° 右转。矫正应发生在"转完并停止转动"之后，而非转弯中途。
    path=[(0.0,0.0),(10.0,0.0),(10.0,10.0)]
    tc=TurnCorrector(path, threshold_deg=35, corner_tolerance=8, cooldown_dist=1)
    assert tc.on_pose({'x':9.0,'y':0.0,'yaw':0.0}) is None        # 直行，未触发
    assert tc.on_pose({'x':10.0,'y':0.0,'yaw':0.3}) is None       # 刚开始转
    # 转过约 90° 的那一刻只进入"转弯中"，不立即矫正（避免中途矫正导致过转）
    assert tc.on_pose({'x':10.0,'y':0.0,'yaw':1.57}) is None
    # 车辆已停止转动 -> 出弯瞬间矫正
    corr=tc.on_pose({'x':10.0,'y':0.0,'yaw':1.57})
    assert corr is not None and abs(corr['yaw']-math.pi/2)<1e-6   # 航向对齐到 90°
    assert corr['pos']==(10.0,0.0)                                # 位置吸附到转角

def test_ekf_turn_correction_flow():
    # 端到端：规划路径 -> EKF 带转弯矫正，沿路径行驶应完成多次矫正
    m=ParkingMap.load(ROOT/'maps/demo_map.json'); pl=LanePlanner(m)
    path,L,side,spot=pl.plan_to_parking('入口A','P031')
    cfg={'uwb_enabled':False,'turn_correction':{'enabled':True,'threshold_deg':35,'corner_tolerance':6,'snap_position':True,'cooldown_dist':2.0}}
    f=ImuUwbEkf(cfg); f.set_path(path); f.reset(*path[0],0)
    tc=f.correction_stats(); assert tc['enabled'] is True

def _wrap(a): return (a+math.pi)%(2*math.pi)-math.pi
def _drive(path, tc_on, seed=7):
    rnd=random.Random(seed)
    def heading(a,b): return math.atan2(b[1]-a[1],b[0]-a[0])
    cfg={'uwb_enabled':False,'turn_correction':{'enabled':tc_on,'threshold_deg':35,'corner_tolerance':6,'snap_position':True,'cooldown_dist':2.0}}
    f=ImuUwbEkf(cfg); f.set_path(path); f.reset(*path[0],heading(path[0],path[1]))
    t=0.0; dt=0.02; seg=0; along=0.0; speed=2.0; last_yaw=heading(path[0],path[1]); max_err=0.0
    while seg<len(path)-1:
        t+=dt; Lseg=dist(path[seg],path[seg+1]); along+=speed*dt
        if along>=Lseg:
            along-=Lseg; seg+=1
            if seg>=len(path)-1: break
        a,b=path[seg],path[min(seg+1,len(path)-1)]; r=min(1.0,along/max(Lseg,1e-9))
        px,py=a[0]+r*(b[0]-a[0]),a[1]+r*(b[1]-a[1]); yaw=heading(a,b)
        gyro=_wrap(yaw-last_yaw)/dt+0.012+rnd.gauss(0,0.015); last_yaw=yaw
        v=f.pose()['speed']; ax=max(-5.0,min(5.0,(speed-v)/dt))+rnd.gauss(0,0.05)
        f.predict(t,ax,gyro); p=f.pose()
        max_err=max(max_err,math.hypot(p['x']-px,p['y']-py))
    return f.correction_stats(), max_err

def test_turn_correction_e2e():
    # 无 UWB：Z 形路径(两个90°转弯)开启矫正应显著抑制累计误差且触发≥2次；直线路径不应误触发
    zpath=[(0.0,0.0),(10.0,0.0),(10.0,10.0),(0.0,10.0)]
    s_on,e_on=_drive(zpath,True); s_off,e_off=_drive(zpath,False)
    assert s_on['count']>=2 and e_off>e_on
    s_straight,_=_drive([(0.0,0.0),(30.0,0.0)],True)
    assert s_straight['count']==0

def _drive_turn_gradual(bias=0.012, seed=7):
    """用连续时间航向曲线驱动 EKF 通过一个"逐渐执行"的 90° 转弯（复现真实陀螺场景）。
    旧逻辑在累计转角达到阈值(35°)时立即把航向摆到90°，随后角速度再积分 → 出弯过转到 ~145°。"""
    rnd=random.Random(seed)
    path=[(0.0,0.0),(10.0,0.0),(10.0,10.0)]
    cfg={'uwb_enabled':False,'turn_correction':{'enabled':True,'threshold_deg':35,'corner_tolerance':8,'snap_position':True,'cooldown_dist':1.0,'align_deg':6.0,'stop_rate_deg':1.5}}
    f=ImuUwbEkf(cfg); f.set_path(path); f.reset(9.0,0.0,0.0)
    dt=0.02
    def real_yaw(t):
        if t<0.5: return 0.0
        if t<1.5: return math.radians(90.0*(t-0.5))   # 0.5~1.5s 从 0 匀速转到 90°
        return math.radians(90.0)
    last=None
    for i in range(200):   # 共 4s，含过弯与出弯后直行
        t=i*dt; yaw=real_yaw(t)
        gyro=0.0 if last is None else _wrap(yaw-last)/dt + bias + rnd.gauss(0,0.015)
        last=yaw
        v=f.pose()['speed']; ax=max(-5.0,min(5.0,(2.0-v)/dt))
        f.predict(t,ax,gyro)
    return f.pose()

def test_turn_correction_gradual_no_overshoot():
    p=_drive_turn_gradual()
    err=math.degrees(_wrap(p['yaw']-math.pi/2))
    # 出弯航向应接近 90°（允许几度），而不是过转到 ~145° 形成斜向漂移
    assert abs(err)<8.0, f"出弯航向 {math.degrees(p['yaw']):.1f}°，过转 {err:.1f}° 应接近 90°"

if __name__=='__main__':
    test_plan(); test_filter(); test_imu_only(); test_plan_to_parking(); test_turn_correction(); test_ekf_turn_correction_flow(); test_turn_correction_e2e(); test_turn_correction_gradual_no_overshoot()
    print('tests passed')
