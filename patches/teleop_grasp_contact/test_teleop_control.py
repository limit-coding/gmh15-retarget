"""Hardware-free tests of motion boundaries and command ownership."""
import time
import unittest
from teleop_control import Controller,angles11,EDITABLE,ReadOnlyHandProxy,AuditedHandProxy

OWNER='test-browser-owner-00001'

class FakeHand:
    def __init__(self):self.enabled=False;self.calls=[];self.faults=[];self.fail_seed=False;self.fail_target=False;self.init_result=1
    def isMotorEnabled(self,a):return self.enabled
    def checkHandStatus(self,a):return 1
    def getMotorControlMode(self,a):return 'POSITION'
    def getFaultCode(self,a):return self.faults
    def getMotorMaxVelocityConfig(self,a):return 'old-speeds'
    def setMotorControlMode(self,a,v):self.calls.append(('mode',v));return True
    def setMotorMaxVelocity(self,a,v):self.calls.append(('speed',v));return True
    def setMotorTargetPosition(self,a,v):self.calls.append(('seed',v));return 0 if self.fail_seed or not self.enabled else 1
    def enableMotor(self,a):self.calls.append(('enable',));self.enabled=True;return True
    def disableMotor(self,a):self.calls.append(('disable',));self.enabled=False;return True
    def setJointPositionsAngle(self,a,v):self.calls.append(('joint-target',v));return 0 if self.fail_target else 1
    def initMotorPosition(self,a):self.calls.append(('initialize',));self.enabled=True;return self.init_result

class FakeKin:
    def calculateNormalizeAngle(self,values,hw):
        return 1,[v/80*1000 if i<8 else (v/109.6*1000-500 if i==8 else v/(87 if i==9 else 120.1)*1000) for i,v in enumerate(values)]

class ControlTests(unittest.TestCase):
    def setUp(self):
        self.now=10.;self.hand=FakeHand()
        self.ctrl=Controller(self.hand,1,FakeKin(),'left','E','POSITION',list,lambda v:('speed',v),clock=lambda:self.now)
        self.frame=dict(connected=True,joint_error=None,timestamp=time.time(),joints_degrees=[0.]*15,motor_positions=[1234]*7)
        self.frame['joints_degrees'][12]=54.8
    def arm(self):self.ctrl.last_init_result=1;self.ctrl.arm(OWNER,self.frame)
    def advance(self,seconds=.11):self.now+=seconds;self.frame['timestamp']=time.time()
    def test_default_does_not_write(self):self.ctrl.tick(self.frame);self.assertEqual(self.hand.calls,[])
    def test_arm_requires_reported_successful_initialization(self):
        with self.assertRaises(ValueError):self.ctrl.arm(OWNER,self.frame)
        self.assertEqual(self.hand.calls,[])
    def test_input_format_and_finite_limits(self):
        for values in ([0]*15,[0]*10,[float('nan')]*11,[True]*11,[1000]*11,[-1]+[0]*10):
            with self.subTest(values=values),self.assertRaises(ValueError):angles11(values)
    def test_sdk_requires_enable_before_target_and_no_zero_seed(self):
        self.arm();names=[x[0] for x in self.hand.calls]
        self.assertLess(names.index('enable'),names.index('seed'))
        self.assertEqual(next(c[1] for c in self.hand.calls if c[0]=='seed'),[1234]*7)
    def test_reject_already_enabled_without_disabling_other_owner(self):
        self.hand.enabled=True
        with self.assertRaises(ValueError):self.arm()
        self.assertEqual(self.hand.calls,[])
    def test_invalid_feedback_cannot_arm(self):
        self.frame['joint_error']='invalid'
        with self.assertRaises(ValueError):self.arm()
        self.assertEqual(self.hand.calls,[])
    def test_seed_failure_disables_after_required_enable(self):
        self.hand.fail_seed=True
        with self.assertRaises(RuntimeError):self.arm()
        self.assertIn(('disable',),self.hand.calls);self.assertFalse(self.hand.enabled);self.assertFalse(self.ctrl.active)
    def test_owner_and_sequence(self):
        self.arm();target=self.ctrl.target[:]
        with self.assertRaises(ValueError):self.ctrl.set_target('different-owner',1,target)
        self.ctrl.set_target(OWNER,1,target)
        with self.assertRaises(ValueError):self.ctrl.set_target(OWNER,1,target)
    def test_rate_and_slew_and_normalized_units(self):
        self.arm();target=self.ctrl.target[:];target[0]=30
        self.ctrl.set_target(OWNER,1,target);self.ctrl.tick(self.frame)
        self.assertEqual(self.ctrl.command_count,0)
        self.advance();self.ctrl.heartbeat(OWNER);self.ctrl.tick(self.frame)
        self.assertLessEqual(self.ctrl.sent[0],.89)
        command=next(c for c in self.hand.calls if c[0]=='joint-target')
        self.assertAlmostEqual(command[1][0],self.ctrl.sent[0]/80*1000)
    def test_deadman_timeout_disables_and_restores(self):
        self.arm();self.advance(.61);self.ctrl.tick(self.frame)
        self.assertFalse(self.hand.enabled);self.assertFalse(self.ctrl.active)
        self.assertEqual(self.hand.calls[-1],('speed','old-speeds'))
    def test_foreign_heartbeat_cannot_keep_control_alive(self):
        self.arm();self.advance(.5);self.ctrl.heartbeat('someone-else');self.advance(.2);self.ctrl.tick(self.frame)
        self.assertFalse(self.ctrl.active)
    def test_fault_disables(self):
        self.arm();self.hand.faults=['fault'];self.advance();self.ctrl.tick(self.frame)
        self.assertFalse(self.hand.enabled)
    def test_lost_feedback_disables(self):
        self.arm();self.frame['connected']=False;self.advance();self.ctrl.tick(self.frame)
        self.assertFalse(self.hand.enabled)
    def test_sdk_rejected_target_disables(self):
        self.arm();self.hand.fail_target=True;self.advance();self.ctrl.tick(self.frame)
        self.assertFalse(self.hand.enabled)
    def test_tracking_deviation_disables(self):
        self.arm();self.frame['joints_degrees'][0]=20;self.advance();self.ctrl.tick(self.frame)
        self.assertFalse(self.hand.enabled)
    def test_readonly_proxy_blocks_every_mutation_before_sdk(self):
        proxy=ReadOnlyHandProxy(self.hand)
        for method in ('enableMotor','disableMotor','setMotorTargetPosition','setJointPositionsAngle',
                       'setMotorControlMode','setMotorMaxVelocity','initMotorPosition','calibrateSensor'):
            with self.subTest(method=method),self.assertRaises(RuntimeError):getattr(proxy,method)
        self.assertEqual(proxy.blocked_write_attempts,8);self.assertEqual(self.hand.calls,[])
    def test_readonly_proxy_permits_enable_state_read(self):
        proxy=ReadOnlyHandProxy(self.hand)
        self.assertFalse(proxy.isMotorEnabled(1));self.assertEqual(proxy.blocked_write_attempts,0)
    def test_initialization_only_on_explicit_call_and_always_disables(self):
        self.assertEqual(self.ctrl.initialization_count,0)
        self.ctrl.initialize(OWNER)
        self.assertEqual(self.ctrl.last_init_result,1)
        self.assertEqual(self.ctrl.initialization_count,1)
        self.assertFalse(self.hand.enabled);self.assertFalse(self.ctrl.active);self.assertFalse(self.ctrl.initializing)
        self.assertEqual(self.hand.calls[-1],('disable',))
        self.assertFalse(any(c[0]=='speed' for c in self.hand.calls))
    def test_failed_initialization_still_disables(self):
        self.hand.init_result=0;self.ctrl.initialize(OWNER)
        self.assertEqual(self.ctrl.last_init_result,0);self.assertIsNotNone(self.ctrl.error)
        self.assertFalse(self.hand.enabled);self.assertFalse(self.ctrl.active)
    def test_init_rejects_external_enabled_hand(self):
        self.hand.enabled=True
        with self.assertRaises(ValueError):self.ctrl.initialize(OWNER)
        self.assertEqual(self.hand.calls,[])
    def test_audited_proxy_starts_at_zero_and_counts_only_explicit_sdk_calls(self):
        proxy=AuditedHandProxy(self.hand)
        self.assertFalse(proxy.isMotorEnabled(1));self.assertEqual(proxy.write_counts,{})
        proxy.initMotorPosition(1)
        self.assertEqual(proxy.write_counts,{'initMotorPosition':1})
        with self.assertRaises(RuntimeError):proxy.calibrateSensor(1)

class GraspTests(ControlTests):
    """Contact-aware tracking: a blocked finger in contact holds instead of disabling."""
    def setUp(self):
        super().setUp()
        self.frame['fingers']=[dict(key=k,force=[0.,0.,.7 if k=='thumb' else .1]) for k in ('index','middle','ring','pinky','thumb')]
    def force(self,key,n):
        for f in self.frame['fingers']:
            if f['key']==key:f['force']=[0.,0.,n]
    def run_ticks(self,n):
        for _ in range(n):self.advance();self.ctrl.heartbeat(OWNER);self.ctrl.tick(self.frame)
    def close_index(self,deg=40):
        target=self.ctrl.target[:];target[0]=deg;self.ctrl.set_target(OWNER,1,target)
    def test_blocked_without_contact_still_disables(self):
        self.arm();self.close_index();self.run_ticks(20)   # measured index stays 0
        self.assertFalse(self.ctrl.active);self.assertFalse(self.hand.enabled)
    def test_blocked_with_contact_holds_and_bounds_lead(self):
        self.arm();self.close_index();self.run_ticks(5);self.force('index',2.);self.run_ticks(40)
        self.assertTrue(self.ctrl.active,self.ctrl.error);self.assertTrue(self.hand.enabled)
        self.assertLessEqual(self.ctrl.sent[0],6.+1e-9);self.assertIn(0,self.ctrl.status()['grasp_contact'])
    def test_contact_cap_follows_measured_angle(self):
        self.frame['joints_degrees'][0]=20.;self.arm();self.close_index();self.force('index',2.);self.run_ticks(60)
        self.assertTrue(self.ctrl.active,self.ctrl.error);self.assertAlmostEqual(self.ctrl.sent[0],26.)
    def test_opening_not_restricted_in_contact(self):
        self.frame['joints_degrees'][0]=20.;self.arm();self.force('index',2.);self.ctrl.sent[0]=24.
        target=self.ctrl.target[:];target[0]=0.;self.ctrl.set_target(OWNER,1,target)
        self.run_ticks(1);self.assertLess(self.ctrl.sent[0],24.)
    def test_high_force_stops_further_closing(self):
        self.arm();self.close_index();self.force('index',2.);self.run_ticks(3);held=self.ctrl.sent[0]
        self.force('index',20.);self.run_ticks(10)
        self.assertTrue(self.ctrl.active);self.assertLessEqual(self.ctrl.sent[0],held+1e-9)
    def test_resting_thumb_force_is_baseline_not_contact(self):
        self.arm();self.run_ticks(2);self.assertEqual(self.ctrl.status()['grasp_contact'],{})
    def test_contact_on_other_finger_does_not_shield_index(self):
        self.arm();self.close_index();self.force('middle',3.);self.run_ticks(20)
        self.assertFalse(self.ctrl.active)
    def test_missing_tactile_falls_back_to_original_rule(self):
        del self.frame['fingers'];self.arm();self.close_index();self.run_ticks(20)
        self.assertFalse(self.ctrl.active)

if __name__=='__main__':unittest.main(verbosity=2)
