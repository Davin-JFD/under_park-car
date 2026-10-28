# 数据接口速查

## IMU UDP 19001

`{"type":"imu","timestamp":1789780000.123,"ax":0.12,"gyro_z":0.008}`

- `ax`: 车体 x 前向线加速度，m/s²，已去重力
- `gyro_z`: 车体 z 轴角速度，rad/s，逆时针为正
- 推荐频率：50~100 Hz

## UWB UDP 19002

`{"type":"uwb_position","timestamp":1789780000.2,"x":23.45,"y":18.72,"std":0.45,"quality":0.92}`

- `x,y`: 地图坐标，m
- `std`: 位置单轴标准差，m
- `quality`: 0~1
- 推荐频率：5~20 Hz

## 坐标约定

地图、UWB、融合输出都使用同一个局部 ENU 平面坐标系；x 向右/东，y 向上/北，yaw 从 x 轴起逆时针为正。
