from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Any
import json

Point = Tuple[float, float]

@dataclass
class Lane:
    id: str
    points: List[Point]
    width: float = 3.5
    bidirectional: bool = False

@dataclass
class ParkingSpot:
    id: str
    position: Point
    rotation_deg: float = 0.0
    width: float = 2.5
    length: float = 5.2
    kind: str = "car"

@dataclass
class NamedPoint:
    id: str
    position: Point
    kind: str = "entrance"

@dataclass
class Anchor:
    id: str
    position: Point
    z: float = 0.0

@dataclass
class ParkingMap:
    name: str = "parking"
    coordinate_system: str = "LOCAL_ENU_METERS"
    lanes: List[Lane] = field(default_factory=list)
    parking_spots: List[ParkingSpot] = field(default_factory=list)
    entrances: List[NamedPoint] = field(default_factory=list)
    obstacles: List[List[Point]] = field(default_factory=list)
    anchors: List[Anchor] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def named_positions(self) -> Dict[str, Point]:
        result = {p.id: p.position for p in self.entrances}
        result.update({p.id: p.position for p in self.parking_spots})
        return result

    def to_dict(self):
        return {
            "schema_version": "1.0",
            "name": self.name,
            "coordinate_system": self.coordinate_system,
            "metadata": self.metadata,
            "lane_network": {"lanes": [
                {"id": x.id, "points": [list(p) for p in x.points], "width": x.width,
                 "bidirectional": x.bidirectional} for x in self.lanes]},
            "parking_spots": [
                {"id": x.id, "position": list(x.position), "rotation_deg": x.rotation_deg,
                 "width": x.width, "length": x.length, "type": x.kind} for x in self.parking_spots],
            "entrances": [{"id": x.id, "position": list(x.position), "type": x.kind} for x in self.entrances],
            "static_obstacles": [{"id": f"OBS-{i+1}", "polygon": [list(p) for p in poly]}
                                 for i, poly in enumerate(self.obstacles)],
            "uwb_anchors": [{"id": x.id, "position": [x.position[0], x.position[1], x.z]}
                            for x in self.anchors]
        }

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, ensure_ascii=False, indent=2)

    @staticmethod
    def load(path):
        with open(path, "r", encoding="utf-8-sig") as f:
            d = json.load(f)
        lanes = [Lane(str(x["id"]), [tuple(map(float, p[:2])) for p in x["points"]],
                      float(x.get("width", 3.5)), bool(x.get("bidirectional", False)))
                 for x in d.get("lane_network", {}).get("lanes", [])]
        spots = [ParkingSpot(str(x["id"]), tuple(map(float, x["position"][:2])),
                             float(x.get("rotation_deg", 0)), float(x.get("width", 2.5)),
                             float(x.get("length", 5.2)), str(x.get("type", "car")))
                 for x in d.get("parking_spots", [])]
        entrances = [NamedPoint(str(x["id"]), tuple(map(float, x["position"][:2])),
                                str(x.get("type", "entrance"))) for x in d.get("entrances", [])]
        obstacles = [[tuple(map(float, p[:2])) for p in x.get("polygon", [])]
                     for x in d.get("static_obstacles", [])]
        anchors = [Anchor(str(x["id"]), tuple(map(float, x["position"][:2])),
                          float(x["position"][2]) if len(x["position"]) > 2 else 0.0)
                   for x in d.get("uwb_anchors", [])]
        return ParkingMap(str(d.get("name", "parking")), str(d.get("coordinate_system", "LOCAL_ENU_METERS")),
                          lanes, spots, entrances, obstacles, anchors, d.get("metadata", {}))
