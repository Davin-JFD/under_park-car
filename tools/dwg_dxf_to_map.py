"""DWG/DXF 到导航地图 JSON 半自动转换器；DWG 需要 ODA File Converter。"""
import argparse, sys
from pathlib import Path
import ezdxf
from ezdxf.addons import odafc
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from parking_nav.models import ParkingMap,Lane,ParkingSpot,NamedPoint

def attrs(e):
    out={}
    try:
        for a in e.attribs: out[a.dxf.tag.strip().upper()]=a.dxf.text.strip()
    except Exception: pass
    return out

def convert(src,out,scale,lane_layers,slot_layers,entrance_layers,bidirectional):
    doc=odafc.readfile(str(src)) if src.suffix.lower()=='.dwg' else ezdxf.readfile(str(src));msp=doc.modelspace();lanes=[];spots=[];ents=[]
    ll={x.upper() for x in lane_layers};sl={x.upper() for x in slot_layers};el={x.upper() for x in entrance_layers}
    for i,e in enumerate(msp):
        layer=e.dxf.layer.upper(); typ=e.dxftype()
        if layer in ll and typ in ('LWPOLYLINE','POLYLINE','LINE'):
            if typ=='LINE':pts=[(e.dxf.start.x*scale,e.dxf.start.y*scale),(e.dxf.end.x*scale,e.dxf.end.y*scale)]
            elif typ=='LWPOLYLINE':pts=[(p[0]*scale,p[1]*scale) for p in e.get_points('xy')]
            else:pts=[(v.dxf.location.x*scale,v.dxf.location.y*scale) for v in e.vertices]
            if len(pts)>1:lanes.append(Lane(f'LANE-{i+1}',pts,3.5,bidirectional))
        elif layer in sl and typ=='INSERT':
            a=attrs(e);spots.append(ParkingSpot(a.get('ID',a.get('NUMBER',str(len(spots)+1))),(float(e.dxf.insert.x)*scale,float(e.dxf.insert.y)*scale),float(e.dxf.rotation or 0)))
        elif layer in el and typ=='INSERT':
            a=attrs(e);ents.append(NamedPoint(a.get('ID',f'ENT-{len(ents)+1}'),(float(e.dxf.insert.x)*scale,float(e.dxf.insert.y)*scale)))
    ParkingMap(src.stem,'LOCAL_ENU_METERS',lanes,spots,ents,[],[],{'source':str(src),'scale_to_meter':scale}).save(out)
    print(f'完成: lanes={len(lanes)}, spots={len(spots)}, entrances={len(ents)} -> {out}')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('input');p.add_argument('output');p.add_argument('--scale',type=float,default=1.0,help='CAD单位到米，如毫米=0.001')
    p.add_argument('--lane-layer',action='append',default=[]);p.add_argument('--slot-layer',action='append',default=[]);p.add_argument('--entrance-layer',action='append',default=[]);p.add_argument('--bidirectional',action='store_true')
    a=p.parse_args();convert(Path(a.input),Path(a.output),a.scale,a.lane_layer or ['LANE_CENTER'],a.slot_layer or ['PARKING_SLOT'],a.entrance_layer or ['ENTRANCE'],a.bidirectional)
