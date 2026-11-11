"""Official gesture adapter preserves measured limits and rejects bad IK before writes."""
import importlib.util
from pathlib import Path

import pytest

spec=importlib.util.spec_from_file_location('gesture_adapter',Path(__file__).resolve().parents[1]/'mac_driver/gesture_adapter.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class Hand:
    def __init__(self):self.writes=[]
    def setMotorTargetPosition(self,address,values):
        self.writes.append(values);return 1


class Kin:
    def calculateRealAngle(self,values,hw):return 1,[0]*11
    def inverseKinematicParse(self,side,values,hw):return 1,[-10,20000,5000,0,10000,9053,9220]


def test_measured_thumb_limits_apply_to_official_gestures():
    hand=Hand();proxy=module.BoundedGestureHand(hand,Kin(),'right','E ',list,lambda *a,**k:None)
    assert proxy.setJointPositionsAngle(1,[0]*11)==1
    assert hand.writes==[[0,10000,5000,0,10000,7200,7700]]


def test_invalid_ik_stops_before_any_motor_command():
    kin=Kin();kin.inverseKinematicParse=lambda *a:(0,[])
    hand=Hand();proxy=module.BoundedGestureHand(hand,kin,'right','E ',list,lambda *a,**k:None)
    with pytest.raises(RuntimeError):proxy.setJointPositionsAngle(1,[0]*11)
    assert hand.writes==[]
