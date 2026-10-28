import argparse,json,socket,time
p=argparse.ArgumentParser()
# 目标IP改为本机网卡IP 10.16.135.215
p.add_argument('--host',default='10.16.135.215');
# IMU端口改为GUI监听的2177
p.add_argument('--imu-port',type=int,default=2177);
# 这里根据GUI的UWB监听端口修改，示例先用2178，你按需改
p.add_argument('--uwb-port',type=int,default=2178);
a=p.parse_args();
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);
t0=time.time();n=0
print('发送测试数据，Ctrl+C 停止')
try:
    while True:
        t=time.time()
        # IMU数据包发送到 10.16.135.215:2177
        s.sendto(json.dumps({'type':'imu','timestamp':t,'ax':0.0,'gyro_z':0.0}).encode(),(a.host,a.imu_port));n+=1
        if n%10==0:
            s.sendto(json.dumps({'type':'uwb_position','timestamp':t,'x':5+(t-t0)*0.5,'y':10,'std':0.5,'quality':1.0}).encode(),(a.host,a.uwb_port))
        time.sleep(.02)
except KeyboardInterrupt:
    print("\n发包停止")
finally:
    s.close()
