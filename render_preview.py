"""Render parking map to PNG using offscreen Qt."""
import sys, os
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PySide6.QtWidgets import QApplication
from parking_nav.canvas import MapCanvas
from parking_nav.models import ParkingMap

app = QApplication(sys.argv)
m = ParkingMap.load('maps/map.json')
canvas = MapCanvas()
canvas.resize(1400, 900)
canvas.set_map(m)
canvas.grab().save('map_preview.png')
print('Saved map_preview.png')
