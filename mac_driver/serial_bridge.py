"""Authenticated single-client raw USB CDC transport. No motor commands here."""
import hmac
import os
import time
import select
import socket
import threading

import serial
from serial.tools.list_ports import comports


def discover():
    return [p.device for p in comports() if (p.vid, p.pid) == (0x0483, 0x5740)]


class SerialBridge:
    def __init__(self, device, token, port=9872, host='127.0.0.1'):
        self.token = token
        self.closed = threading.Event()
        self.error = None
        self.tx = self.rx = 0
        self.connection = None
        # Optional diagnostic capture: GMH15_USB_TRACE=/path/file.txt (timestamp, direction, hex).
        trace = os.environ.get('GMH15_USB_TRACE')
        self.trace = open(trace, 'a', buffering=1) if trace else None
        # STM32 native USB CDC is a byte stream; UART baud does not set USB speed.
        self.serial = serial.Serial(device, 115200, timeout=0, write_timeout=1, exclusive=True)
        self.listener = socket.socket()
        try:
            self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.listener.bind((host, port))
            self.listener.listen(1)
            self.listener.settimeout(.2)
        except BaseException:
            self.serial.close()
            self.listener.close()
            raise
        self.port = self.listener.getsockname()[1]

    def serve(self):
        try:
            while not self.closed.is_set():
                try:
                    conn, _ = self.listener.accept()
                except socket.timeout:
                    continue
                with conn:
                    self.connection = conn
                    conn.settimeout(2)
                    auth = bytearray()
                    while len(auth) < 128:
                        byte = conn.recv(1)
                        if not byte or byte == b'\n':
                            break
                        auth.extend(byte)
                    if not hmac.compare_digest(bytes(auth), self.token.encode()):
                        continue
                    conn.sendall(b'OK\n')
                    conn.settimeout(1)
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    while not self.closed.is_set():
                        ready, _, _ = select.select([conn, self.serial.fileno()], [], [], .2)
                        if conn in ready:
                            data = conn.recv(65536)
                            if not data:
                                break
                            self.serial.write(data)
                            self.tx += len(data)
                            self._log('>', data)
                        if self.serial.fileno() in ready:
                            data = self.serial.read(65536)
                            if data:
                                conn.sendall(data)
                                self.rx += len(data)
                                self._log('<', data)
                    # A session ends on disconnect. Do not replay stale serial data.
                    return
        except (OSError, serial.SerialException) as exc:
            if not self.closed.is_set():
                self.error = str(exc)
        finally:
            self.closed.set()
            self.serial.close()
            self.listener.close()

    def _log(self, direction, data):
        if self.trace:
            self.trace.write(f'{time.monotonic():.6f} {direction} {data.hex()}\n')

    def close(self):
        self.closed.set()
        if self.connection:
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

