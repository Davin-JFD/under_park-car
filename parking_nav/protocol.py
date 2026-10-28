import json, time

def parse_packet(raw: bytes, expected: str):
    d=json.loads(raw.decode("utf-8-sig"))
    if d.get("type") != expected: raise ValueError(f"需要 type={expected}")
    d["timestamp"]=float(d.get("timestamp", time.time()))
    if expected == "imu":
        d["ax"]=float(d["ax"]); d["gyro_z"]=float(d["gyro_z"])
    elif expected == "uwb_position":
        d["x"]=float(d["x"]); d["y"]=float(d["y"])
        d["std"]=float(d.get("std",0.6)); d["quality"]=float(d.get("quality",1.0))
    return d
