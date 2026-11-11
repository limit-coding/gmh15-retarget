"""Real PTY/socket checks for the new Mac transport; no robot involved."""
import importlib.util
import os
from pathlib import Path
import pty
import socket
import threading
import time
import tty


spec = importlib.util.spec_from_file_location('serial_bridge', Path(__file__).resolve().parents[1] / 'mac_driver/serial_bridge.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_authenticated_transport_rejects_bad_clients_and_preserves_binary_bytes():
    master, slave = pty.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    bridge = module.SerialBridge(os.ttyname(slave), 'test-secret', port=0)
    worker = threading.Thread(target=bridge.serve, daemon=True)
    worker.start()
    try:
        with socket.create_connection(('127.0.0.1', bridge.port), timeout=2) as client:
            client.sendall(b'incorrect-token\n')
            assert client.recv(1) == b''
        assert bridge.tx == bridge.rx == 0
        with socket.create_connection(('127.0.0.1', bridge.port), timeout=2) as client:
            client.sendall(b'test-secret\n')
            assert client.recv(3) == b'OK\n'
            payload = b'\x00\r\n\xff\x11\x13binary'
            client.sendall(payload)
            import select
            assert select.select([master], [], [], 2)[0]
            assert os.read(master, 1024) == payload
            os.write(master, payload[::-1])
            received = bytearray()
            while len(received) < len(payload):
                received.extend(client.recv(1024))
            assert bytes(received) == payload[::-1]
        worker.join(timeout=2)
        assert not worker.is_alive()
        assert bridge.closed.is_set()
        assert bridge.error is None
        assert bridge.tx == bridge.rx == len(payload)
    finally:
        bridge.close()
        worker.join(timeout=2)
        os.close(master)
        os.close(slave)
