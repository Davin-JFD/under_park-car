from __future__ import annotations
import socket, threading
from PySide6.QtCore import QObject, Signal
from .protocol import parse_packet

class UdpReceiver(QObject):
    packet = Signal(dict); status = Signal(str)
    def __init__(self, host, port, packet_type):
        super().__init__(); self.host=host; self.port=int(port); self.packet_type=packet_type
        self.running=False; self.sock=None
    def start(self):
        if self.running: return
        self.running=True; threading.Thread(target=self._run, daemon=True).start()
    def stop(self):
        self.running=False
        if self.sock:
            try: self.sock.close()
            except OSError: pass
    def _run(self):
        try:
            self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); self.sock.settimeout(0.5)
            self.sock.bind((self.host,self.port)); self.status.emit(f"{self.packet_type} UDP 已监听 {self.host}:{self.port}")
            while self.running:
                try:
                    raw,_=self.sock.recvfrom(65535); self.packet.emit(parse_packet(raw,self.packet_type))
                except socket.timeout: continue
                except (ValueError,KeyError,UnicodeError) as e: self.status.emit(f"数据丢弃: {e}")
        except OSError as e: self.status.emit(f"UDP 启动失败: {e}")
        finally: self.running=False
