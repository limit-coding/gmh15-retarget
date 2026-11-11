"""Read the camera continuously; consumers take the newest frame without a backlog."""
import threading
import time


class LatestCamera:
    def __init__(self, cap):
        self.cap = cap
        self.condition = threading.Condition()
        self.stopped = threading.Event()
        self.sequence = self.consumed = 0
        self.frame = None
        self.timestamp = 0.
        self.ended = False
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def _run(self):
        try:
            while not self.stopped.is_set():
                ok, frame = self.cap.read()
                if not ok:
                    break
                with self.condition:
                    self.frame, self.timestamp = frame, time.monotonic()
                    self.sequence += 1
                    self.condition.notify_all()
        finally:
            with self.condition:
                self.ended = True
                self.condition.notify_all()

    def read(self):
        with self.condition:
            while self.sequence == self.consumed and not self.ended and not self.stopped.is_set():
                self.condition.wait(.2)
            if self.sequence == self.consumed:
                return False, None, self.timestamp
            self.consumed = self.sequence
            return True, self.frame, self.timestamp

    def close(self):
        self.stopped.set()
        with self.condition:
            self.condition.notify_all()
        self.worker.join(timeout=2)
        self.cap.release()
