"""MuJoCo WASM raw WebSocket pose -> original Newton SciPy retarget -> VaM UDP.
Requires numpy, scipy, websockets. Does NOT require PyTorch or Newton.
"""
import asyncio
import json
import sys
from pathlib import Path
import socket
import struct
import time
import numpy as np
from websockets.asyncio.server import serve
from vam_udp_sender_original import VaMUDPMixin

BODY_COUNT = 31
DOF_COUNT = 29
NUM_FLOATS = BODY_COUNT * (3 + 4) + DOF_COUNT
MAGIC = b'MVR1'
INPUT_SIZE = 8 + NUM_FLOATS * 4
OUTPUT_SIZE = 133 * 4

class PoseEngine:
    """Expose MuJoCo body arrays with the Newton engine getter interface."""
    def __init__(self):
        self.pos = np.zeros((BODY_COUNT, 3), dtype=np.float64)
        self.rot = np.tile([1., 0., 0., 0.], (BODY_COUNT, 1)) # wxyz
        self.dofs = np.zeros(DOF_COUNT, dtype=np.float64)

    def get_body_pos(self, _char_id): return self.pos
    def get_body_rot(self, _char_id): return self.rot
    def get_root_pos(self, _char_id): return self.pos[0]
    def get_root_rot(self, _char_id): return self.rot[0]
    def get_dof_pos(self, _char_id): return self.dofs

class Retarget(VaMUDPMixin):
    def __init__(self, dest_host, dest_port, position_offset=(0., .06, 0.)):
        self._engine = PoseEngine()
        self._vam_ip = dest_host
        self._vam_port = dest_port
        self._vam_position_offset = np.asarray(position_offset, dtype=np.float32)
        self._vam_limb_bind_euler_deg = {}
        self._vam_limb_bind_quat_xyzw = {}
        self._vam_packet = struct.Struct("<133f")
        self._vam_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _get_char_id(self): return 0

    @staticmethod
    def _vam_to_numpy(value): return np.asarray(value)

    def process(self, frame):
        if not isinstance(frame, bytes) or len(frame) != INPUT_SIZE:
            raise ValueError(f'wrong message length: expected {INPUT_SIZE}, got {len(frame) if isinstance(frame, bytes) else type(frame)}')
        if frame[:4] != MAGIC or struct.unpack_from('<I', frame, 4)[0] != NUM_FLOATS * 4:
            raise ValueError('invalid MVR1 header / payload size')
        vals = np.frombuffer(frame, dtype='<f4', offset=8)
        if not np.all(np.isfinite(vals)):
            raise ValueError('non-finite pose values')
        p = 0
        self._engine.pos = vals[p:p+BODY_COUNT*3].reshape(BODY_COUNT, 3); p += BODY_COUNT*3
        self._engine.rot = vals[p:p+BODY_COUNT*4].reshape(BODY_COUNT, 4); p += BODY_COUNT*4
        self._engine.dofs = vals[p:p+DOF_COUNT]
        norms = np.linalg.norm(self._engine.rot, axis=1)
        if not np.all((norms > 0.95) & (norms < 1.05)):
            raise ValueError('invalid body quaternion norms')
        # Exactly the original _send_vam_udp() implementation.
        self._send_vam_udp()

def load_config():
    """Read user-editable JSON beside the EXE (or beside this script in dev)."""
    install_dir = Path(sys.executable).resolve().parent if getattr(sys, 'frozen', False) else Path(__file__).resolve().parent
    config_path = install_dir / 'bridge_config.json'
    if not config_path.is_file():
        raise FileNotFoundError(f'Configuration file not found: {config_path}')
    try:
        config = json.loads(config_path.read_text(encoding='utf-8-sig'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f'Cannot read {config_path}: {exc}') from exc
    if not isinstance(config, dict):
        raise ValueError(f'Config must be a JSON object: {config_path}')
    host = config.get('udp_host')
    if not isinstance(host, str) or not host.strip() or len(host) > 253:
        raise ValueError('bridge_config.json: udp_host must be a nonempty hostname or IP address')
    host = host.strip()
    try:
        socket.getaddrinfo(host, 9998, type=socket.SOCK_DGRAM)
    except socket.gaierror as exc:
        raise ValueError(f'bridge_config.json: Cannot resolve udp_host {host!r}: {exc}') from exc
    print(f'[bridge] Config: {config_path} ; udp_host={host}', flush=True)
    return host


async def main():
    ws_host = '127.0.0.1'
    ws_port = 8765
    udp_host = load_config()
    udp_port = 9998
    if not all(1 <= p <= 65535 for p in (ws_port, udp_port)): raise ValueError('Invalid port')
    bridge = Retarget(udp_host, udp_port)
    count, errors = 0, 0
    clients = set()
    vr_packets = 0
    furniture_packets = 0

    async def vr_listener():
        nonlocal vr_packets
        loop = asyncio.get_running_loop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', 9999))  # VaM VRHand/HMD sender (92-byte '<23f')
        sock.setblocking(False)
        print('[VR] Listening on UDP 0.0.0.0:9999', flush=True)
        try:
            while True:
                payload, _addr = await loop.sock_recvfrom(sock, 2048)
                if len(payload) != 92:
                    continue
                values = np.frombuffer(payload, dtype='<f4')
                if not bool(np.all(np.isfinite(values))):
                    continue
                # Keep only most recent VR sample; send in band on existing WebSocket.
                packet = b'VRH1' + payload
                vr_packets += 1
                for peer in tuple(clients):
                    try:
                        await peer.send(packet)
                    except Exception:
                        clients.discard(peer)
        finally:
            sock.close()


    async def furniture_listener():
        nonlocal furniture_packets
        loop = asyncio.get_running_loop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', 9001))
        sock.setblocking(False)
        print('[Furniture] UDP 0.0.0.0:9001 -> WebSocket ENV1 (pose only)', flush=True)
        try:
            while True:
                payload, _addr = await loop.sock_recvfrom(sock, 65535)
                if len(payload) < 4 or len(payload) % 4:
                    continue
                (n,) = struct.unpack_from('<f', payload)
                if not np.isfinite(n) or n < 0 or n > 1000 or n != int(n) or len(payload) != 4 * (1 + 11 * int(n)):
                    continue
                # Only already-registered XML shapes are used in the browser.
                # No listening on UDP:9002 and no asset mutation.
                packet = b'ENV1' + payload
                furniture_packets += 1
                for peer in tuple(clients):
                    try:
                        await peer.send(packet)
                    except Exception:
                        clients.discard(peer)
        finally:
            sock.close()

    async def handler(ws):
        nonlocal count, errors
        clients.add(ws)
        print('[bridge] WebSocket connected', flush=True)
        try:
            async for packet in ws:
                if not isinstance(packet, bytes):
                    errors += 1; continue
                try:
                    bridge.process(packet)
                    count += 1
                except ValueError as e:
                    errors += 1
                    if errors <= 5 or errors % 100 == 0: print('[bridge] bad frame:', e, flush=True)
        finally:
            clients.discard(ws)
            print('[bridge] disconnected', flush=True)

    async def statistics():
        last = 0
        while True:
            await asyncio.sleep(5)
            print(f'[bridge] frames={count} (+{count-last}/5s), invalid={errors}, VR packets={vr_packets}, Furniture packets={furniture_packets}, UDP={udp_host}:{udp_port}', flush=True)
            last = count

    print(f'[bridge] raw pose ws://{ws_host}:{ws_port} -> Newton SciPy retarget -> udp://{udp_host}:{udp_port}', flush=True)
    async with serve(handler, ws_host, ws_port, max_size=INPUT_SIZE, compression=None):
        task = asyncio.create_task(statistics())
        vr_task = asyncio.create_task(vr_listener())
        furniture_task = asyncio.create_task(furniture_listener())
        try: await asyncio.Future()
        finally:
            task.cancel()
            vr_task.cancel()
            furniture_task.cancel()
            await asyncio.gather(task, vr_task, furniture_task, return_exceptions=True)

if __name__ == '__main__':
    try: asyncio.run(main())
    except KeyboardInterrupt: pass
