"""Connect the Linux SDK's PTY to the Mac USB transport; supervise its lifetime."""
import os
import pty
import select
import signal
import socket
import subprocess
import sys
import threading
import tty


def main():
    token = os.environ['GMH15_BRIDGE_TOKEN']
    conn = socket.create_connection(('host.docker.internal', int(os.environ.get('GMH15_BRIDGE_PORT', 9872))), timeout=5)
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    conn.sendall(token.encode() + b'\n')
    reply = b''
    while len(reply) < 3:
        part = conn.recv(3 - len(reply))
        if not part:
            break
        reply += part
    if reply != b'OK\n':
        raise RuntimeError('Mac USB bridge authentication failed')
    conn.settimeout(1)
    master, slave = pty.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    path = '/tmp/gmh15-serial'
    if os.path.lexists(path):
        os.unlink(path)
    os.symlink(os.ttyname(slave), path)
    child = subprocess.Popen(sys.argv[1:], start_new_session=True)
    stopped = threading.Event()

    def stop(*_):
        stopped.set()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        while child.poll() is None and not stopped.is_set():
            ready, _, _ = select.select([master, conn], [], [], .1)
            if master in ready:
                data = os.read(master, 65536)
                if not data:
                    break
                conn.sendall(data)
            if conn in ready:
                data = conn.recv(65536)
                if not data:
                    raise ConnectionError('Mac USB bridge disconnected')
                view = memoryview(data)
                while view:
                    view = view[os.write(master, view):]
    finally:
        if child.poll() is None:
            # Let the HTTP parent set the workers' stop events and disable motors.
            # Signalling its entire group here would kill workers before cleanup.
            child.send_signal(signal.SIGTERM)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        os.close(master)
        os.close(slave)
        conn.close()
    return child.returncode


if __name__ == '__main__':
    sys.exit(main())
