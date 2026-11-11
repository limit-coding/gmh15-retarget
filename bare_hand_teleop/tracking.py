"""Hand tracking backends. Each returns a list[HandObservation] per frame.

MediaPipeTracker uses the MediaPipe Tasks HandLandmarker (RGB only, CPU-friendly).
Needs `pip install mediapipe` and the model file `hand_landmarker.task`
(https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task).

Handedness: the MediaPipe Tasks label describes the hand as it APPEARS in the image passed in.
Measured on the Mac camera (runs/sim_live2.jsonl): with the frame flipped (selfie view) the
operator's right hand was labelled "Left" in 699/701 frames while its geometry was right-handed
in 99.9%. So with `mirrored=True` the label is swapped back; with `mirrored=False` it is kept.
Verify once on a new mounting — the sim panel's chirality line cross-checks it.
"""
import numpy as np

from .hand_types import HandObservation


class MediaPipeTracker:
    def __init__(self, model_path, num_hands=2, mirrored=True,
                 det_conf=0.5, presence_conf=0.5, track_conf=0.5):
        try:
            import mediapipe as mp
            from mediapipe.tasks import python as mp_python
            from mediapipe.tasks.python import vision
        except ImportError as e:
            raise SystemExit("mediapipe not installed: pip install mediapipe") from e
        self.mp = mp
        self.mirrored = mirrored
        opts = vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO, num_hands=num_hands,
            min_hand_detection_confidence=det_conf, min_hand_presence_confidence=presence_conf,
            min_tracking_confidence=track_conf)
        self.lm = vision.HandLandmarker.create_from_options(opts)
        self._last_ms = -1

    def process(self, frame_bgr, t):
        rgb = np.ascontiguousarray(frame_bgr[:, :, ::-1])
        ms = max(int(t * 1000), self._last_ms + 1)       # VIDEO mode needs strictly increasing ts
        self._last_ms = ms
        res = self.lm.detect_for_video(self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), ms)
        out = []
        for img, world, hd in zip(res.hand_landmarks, res.hand_world_landmarks, res.handedness):
            label, score = hd[0].category_name, float(hd[0].score)
            if self.mirrored:
                label = {"Left": "Right", "Right": "Left"}.get(label, label)
            out.append(HandObservation(
                t=t, detected=True, handedness=label, score=score,
                world=np.array([[p.x, p.y, p.z] for p in world]),
                image=np.array([[p.x, p.y, p.z] for p in img])))
        return out

    def close(self):
        self.lm.close()


def select_hand(observations, t, side=None):
    """Pick the observation for `side` ("Left"/"Right"); None -> highest score.
    If only the other hand is visible it is returned anyway, so the gate reports
    wrong_hand(...) instead of a misleading no_hand."""
    if not observations:
        return HandObservation(t=t, detected=False)
    cands = [o for o in observations if side is None or o.handedness == side]
    return max(cands or observations, key=lambda o: o.score)
