from __future__ import annotations
import math
from PySide6.QtCore import Qt, QPointF, QRectF
from PySide6.QtGui import QColor, QPainter, QPen, QBrush, QPolygonF
from PySide6.QtWidgets import QWidget

class MapCanvas(QWidget):
    def __init__(self):
        super().__init__(); self.setMinimumSize(720,520); self.map=None; self.path=[]; self.trail=[]; self.pose=None; self.uwb=None
        self.goal_spot=None; self.stop_point=None
        self.scale=7.0; self.offset=QPointF(70,70); self.drag=None; self.setMouseTracking(True)
    def set_map(self,m): self.map=m; self.fit_map(); self.update()
    def set_path(self,p): self.path=list(p); self.update()
    def set_goal(self, spot_id, stop_point): self.goal_spot=spot_id; self.stop_point=stop_point; self.update()
    def clear_track(self): self.trail=[]; self.pose=None; self.uwb=None; self.update()
    def update_pose(self,p): self.pose=p; self.trail.append((p['x'],p['y'])); self.trail=self.trail[-3000:]; self.update()
    def update_uwb(self,p): self.uwb=(p['x'],p['y']); self.update()
    def all_points(self):
        if not self.map:return []
        pts=[]
        for x in self.map.lanes: pts+=x.points
        pts += [x.position for x in self.map.parking_spots+self.map.entrances+self.map.anchors]
        for x in self.map.obstacles: pts+=x
        return pts
    def fit_map(self):
        pts=self.all_points()
        if not pts:return
        xs=[p[0] for p in pts]; ys=[p[1] for p in pts]; w=max(max(xs)-min(xs),1); h=max(max(ys)-min(ys),1)
        self.scale=max(1,min((self.width()-100)/w,(self.height()-100)/h)); self.offset=QPointF(50-min(xs)*self.scale,50+max(ys)*self.scale)
    def world(self,p): return QPointF(self.offset.x()+p[0]*self.scale,self.offset.y()-p[1]*self.scale)
    def poly(self,pts): return QPolygonF([self.world(p) for p in pts])
    def wheelEvent(self,e):
        before=e.position(); factor=1.15 if e.angleDelta().y()>0 else 1/1.15; old=self.scale; self.scale=max(.2,min(80,self.scale*factor))
        self.offset=before+(self.offset-before)*(self.scale/old); self.update()
    def mousePressEvent(self,e):
        if e.button()==Qt.LeftButton:self.drag=e.position()
    def mouseMoveEvent(self,e):
        if self.drag is not None:self.offset+=e.position()-self.drag; self.drag=e.position(); self.update()
    def mouseReleaseEvent(self,e): self.drag=None
    def paintEvent(self,e):
        q=QPainter(self); q.setRenderHint(QPainter.Antialiasing); q.fillRect(self.rect(),QColor('#f6f8fb'))
        if not self.map: q.drawText(self.rect(),Qt.AlignCenter,'未加载地图');return
        q.setPen(QPen(QColor('#d32f2f'),2)); q.setBrush(QBrush(QColor(244,67,54,60)))
        for poly in self.map.obstacles:q.drawPolygon(self.poly(poly))
        for lane in self.map.lanes:
            q.setPen(QPen(QColor('#78909c'),max(2,lane.width*self.scale),Qt.SolidLine,Qt.RoundCap,Qt.RoundJoin)); q.drawPolyline(self.poly(lane.points))
            q.setPen(QPen(QColor('#ffffff'),2,Qt.DashLine)); q.drawPolyline(self.poly(lane.points))
            if len(lane.points)>1:
                a=self.world(lane.points[-2]); b=self.world(lane.points[-1]); ang=math.atan2(b.y()-a.y(),b.x()-a.x()); s=9
                q.setBrush(QColor('#1976d2')); q.setPen(Qt.NoPen); q.drawPolygon(QPolygonF([b,QPointF(b.x()-s*math.cos(ang-.5),b.y()-s*math.sin(ang-.5)),QPointF(b.x()-s*math.cos(ang+.5),b.y()-s*math.sin(ang+.5))]))
        for s in self.map.parking_spots:
            c=self.world(s.position); w=s.width*self.scale; h=s.length*self.scale
            q.save(); q.translate(c); q.rotate(-s.rotation_deg); q.setPen(QPen(QColor('#ff8f00'),2)); q.setBrush(QColor(255,193,7,35)); q.drawRect(QRectF(-w/2,-h/2,w,h)); q.restore()
            q.setPen(QColor('#7a4d00')); q.drawText(QRectF(c.x()-30,c.y()-9,60,18),Qt.AlignCenter,s.id)
        if self.goal_spot and self.map:
            gs=next((s for s in self.map.parking_spots if s.id==self.goal_spot),None)
            if gs:
                c=self.world(gs.position)
                q.setPen(QPen(QColor('#c62828'),3)); q.setBrush(Qt.NoBrush); q.drawEllipse(c,16,16)
        if self.stop_point:
            c=self.world(self.stop_point)
            q.setPen(QPen(QColor('#c62828'),3)); q.setBrush(QColor('#c62828')); q.drawEllipse(c,6,6)
            q.setPen(QColor('#c62828')); q.drawText(QRectF(c.x()-45,c.y()-32,90,20),Qt.AlignCenter,'停车点')
        q.setPen(Qt.NoPen)
        for a in self.map.anchors:
            c=self.world(a.position); q.setBrush(QColor('#8e24aa'));q.drawEllipse(c,6,6);q.setPen(QColor('#6a1b9a'));q.drawText(c+QPointF(7,-7),a.id);q.setPen(Qt.NoPen)
        for x in self.map.entrances:
            c=self.world(x.position);q.setBrush(QColor('#00897b'));q.drawEllipse(c,7,7);q.setPen(QColor('#00695c'));q.drawText(c+QPointF(7,-7),x.id);q.setPen(Qt.NoPen)
        if self.path: q.setPen(QPen(QColor('#1565c0'),4,Qt.SolidLine,Qt.RoundCap,Qt.RoundJoin));q.drawPolyline(self.poly(self.path))
        if len(self.trail)>1: q.setPen(QPen(QColor('#43a047'),2));q.drawPolyline(self.poly(self.trail))
        if self.uwb: c=self.world(self.uwb);q.setPen(QPen(QColor('#ef6c00'),2));q.setBrush(QColor(255,152,0,60));q.drawEllipse(c,5,5)
        if self.pose:
            c=self.world((self.pose['x'],self.pose['y'])); q.save();q.translate(c);q.rotate(-math.degrees(self.pose['yaw']));q.setBrush(QColor('#00a651'));q.setPen(QPen(QColor('#004d40'),1));q.drawPolygon(QPolygonF([QPointF(14,0),QPointF(-10,-8),QPointF(-7,0),QPointF(-10,8)]));q.restore()
