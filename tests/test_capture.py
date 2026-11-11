"""Capture keeps the freshest frame, with no robot or physical camera."""
import queue
import threading
import time

from bare_hand_teleop.capture import LatestCamera


class Camera:
    def __init__(self):self.frames=queue.Queue();self.released=False
    def read(self):
        try:
            frame=self.frames.get(timeout=.05)
            return frame is not None,frame
        except queue.Empty:
            return True,'idle'
    def release(self):self.released=True


def test_latest_camera_discards_backlog_and_closes():
    cap=Camera()
    cap.frames.put('first')
    reader=LatestCamera(cap)
    try:
        ok,frame,stamp=reader.read()
        assert ok and stamp>0
        with reader.condition:
            cap.frames.put('second');cap.frames.put('latest')
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            with reader.condition:
                if reader.frame=='latest':break
            time.sleep(.001)
        ok,frame,_=reader.read()
        assert ok and frame=='latest'
    finally:
        reader.close()
    assert cap.released and not reader.worker.is_alive()


def test_camera_failure_unblocks_consumer():
    cap=Camera();cap.frames.put(None)
    reader=LatestCamera(cap)
    try:
        assert reader.read()[0] is False
    finally:
        reader.close()
