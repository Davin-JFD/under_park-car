import argparse,json,socket,time
p=argparse.ArgumentParser();p.add_argument('--host',default='127.0.0.1');p.add_argument('--imu-port',type=int,default=19001);p.add_argument('--uwb-port',type=int,default=19002);a=p.parse_args();s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);t0=time.time();n=0
print('发送测试数据，Ctrl+C 停止')
while True:
    t=time.time();s.sendto(json.dumps({'type':'imu','timestamp':t,'ax':0.0,'gyro_z':0.0}).encode(),(a.host,a.imu_port));n+=1
    if n%10==0:s.sendto(json.dumps({'type':'uwb_position','timestamp':t,'x':5+(t-t0)*0.5,'y':10,'std':0.5,'quality':1.0}).encode(),(a.host,a.uwb_port))
    time.sleep(.02)
