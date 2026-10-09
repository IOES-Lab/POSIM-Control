"""Regressions for a capsized hull being restored inverted and retried forever."""
import math
from pathlib import Path
import sys
import unittest

from posim_control.stability import CapsizeWatch, restore_attitude, tilt_degrees


def attitude(roll=0., yaw=0.):
    return dict(x=math.sin(roll/2)*math.cos(yaw/2),
                y=math.sin(roll/2)*math.sin(yaw/2),
                z=math.cos(roll/2)*math.sin(yaw/2),
                w=math.cos(roll/2)*math.cos(yaw/2))


class CapsizeSafety(unittest.TestCase):
    def test_yaw_does_not_count_as_tilt(self):
        self.assertAlmostEqual(tilt_degrees(attitude(.2, 2.9)), math.degrees(.2))

    def test_sustained_inversion_triggers_with_simulation_time(self):
        watch=CapsizeWatch();q=attitude(math.pi)
        self.assertFalse(watch.update('lead','a',10.,q))
        for _ in range(20):self.assertFalse(watch.update('lead','a',10.,q))
        self.assertFalse(watch.update('lead','a',11.99,q))
        self.assertTrue(watch.update('lead','a',12.,q))

    def test_transient_wave_tilt_pause_and_world_changes_reset_hold(self):
        watch=CapsizeWatch();q=attitude(math.pi/2)
        watch.update('lead','a',0.,q)
        self.assertFalse(watch.update('lead','a',1.,attitude(.3)))
        self.assertFalse(watch.update('lead','a',5.,q))
        self.assertFalse(watch.update('lead','a',9.,q,paused=True))
        self.assertFalse(watch.update('lead','a',12.,q))
        self.assertFalse(watch.update('lead','b',20.,q))
        self.assertFalse(watch.update('lead','b',1.,q))

    def test_follower_is_watched_independently(self):
        watch=CapsizeWatch();q=attitude(math.pi)
        watch.update('follower','a',10.,q)
        self.assertFalse(watch.update('lead','a',12.,attitude()))
        self.assertTrue(watch.update('follower','a',12.,q))

    def test_restore_inverted_hull_drops_unsafe_momentum(self):
        native=dict(local_pose=dict(position=dict(x=44.,y=17.,z=-.8),orientation=attitude(math.pi)),
                    velocity=dict(linear_body=dict(x=12.,y=4.,z=-2.),angular_body=dict(x=5.,y=2.,z=3.)))
        restored=restore_attitude(native,230.)
        self.assertTrue(restored['righted'])
        self.assertEqual(restored['angles'],[0.,0.,math.radians(-140.)])
        self.assertEqual(restored['linear'],[0.,0.,0.])
        self.assertEqual(restored['angular'],[0.,0.,0.])
        # The recovery API still uses the saved geographic XY; no relocated
        # coordinate is returned by this attitude-only correction.
        self.assertNotIn('x',restored)
        self.assertEqual(native['local_pose']['position']['x'],44.)

    def test_normal_restart_retains_physical_motion(self):
        native=dict(local_pose=dict(position=dict(z=.14),orientation=attitude(.1)),
                    velocity=dict(linear_body=dict(x=10.,y=.1,z=.2),angular_body=dict(x=.01,y=.02,z=.03)))
        restored=restore_attitude(native,90.)
        self.assertFalse(restored['righted'])
        self.assertAlmostEqual(restored['angles'][0],.1)
        self.assertEqual(restored['z'],.14)
        self.assertEqual(restored['linear'],[10.,.1,.2])
        self.assertEqual(restored['angular'],[.01,.02,.03])

    def test_invalid_orientation_is_not_hidden_by_recovery(self):
        for q in (dict(x=0.,y=0.,z=0.,w=0.),dict(x=math.nan,y=0.,z=0.,w=1.)):
            with self.assertRaisesRegex(ValueError,'orientation_invalid'):tilt_degrees(q)

    def test_accepted_quaternion_roundoff_is_normalized_on_restore(self):
        q={k:v*1.01 for k,v in attitude(.3).items()}
        native=dict(local_pose=dict(position=dict(z=.1),orientation=q),
                    velocity=dict(linear_body=dict(x=1.,y=0.,z=0.),angular_body=dict(x=0.,y=0.,z=0.)))
        self.assertAlmostEqual(restore_attitude(native,90.)['angles'][0],.3)


if __name__=='__main__':unittest.main()
