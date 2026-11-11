"""Mac AVFoundation camera -> token-protected local JPEG endpoint."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time


class CameraRelay:
    def __init__(self, index, token, port=9873):
        self.index = index
        self.path = '/' + token + '/camera.jpg'
        self.jpeg = None
        self.timestamp = 0
        self.error = 'Camera starting'
        self.closed = threading.Event()
        relay = self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path != relay.path:
                    self.send_error(404)
                    return
                jpeg = relay.jpeg
                if jpeg is None or time.monotonic() - relay.timestamp > 2:
                    self.send_error(503, relay.error or 'Camera frame stale')
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    self.wfile.write(jpeg)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            def log_message(self, *_):
                pass
        self.server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
        self.threads = [threading.Thread(target=self.server.serve_forever, daemon=True),
                        threading.Thread(target=self.capture, daemon=True)]

    def start(self):
        for thread in self.threads:
            thread.start()

    def capture(self):
        import cv2
        cap = cv2.VideoCapture(self.index, cv2.CAP_AVFOUNDATION)
        try:
            if not cap.isOpened():
                raise RuntimeError('Cannot open selected USB camera')
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            while not self.closed.is_set():
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError('USB camera capture failed')
                ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 78])
                if ok:
                    self.jpeg = data.tobytes()
                    self.timestamp = time.monotonic()
                    self.error = None
                self.closed.wait(.1)
        except Exception as exc:
            self.error = str(exc)
            print('Mac camera:', exc, flush=True)
        finally:
            cap.release()

    def close(self):
        self.closed.set()
        self.server.shutdown()
        self.server.server_close()
        for thread in self.threads:
            thread.join(timeout=2)
