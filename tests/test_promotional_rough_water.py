"""Measured rough-water hysteresis and route-based arrival regressions."""
import copy
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
from posim_control.speed import RoughWaterWatch, target_speed
from posim_control.arrival import estimate_seconds


def pose(z=0.,tilt=0.):
 return dict(position=dict(x=0.,y=0.,z=z),orientation=dict(
  x=math.sin(math.radians(tilt)/2),y=0.,z=0.,w=math.cos(math.radians(tilt)/2)))


class RoughWater(unittest.TestCase):
 def test_either_hull_can_reduce_cruise(self):
  watch=RoughWaterWatch()
  value=watch.update('a',0.,dict(lead=pose(),follower=pose(tilt=21.)))
  self.assertTrue(value['rough']);self.assertEqual(value['speed_cap_mps'],7.)
  self.assertEqual(target_speed(False,False,30,rough=value['rough']),7.)

 def test_heave_needs_four_seconds_of_real_observations(self):
  watch=RoughWaterWatch()
  for t in range(4):self.assertFalse(watch.update('a',float(t),dict(lead=pose(z=(-1)**t*.5)))['rough'])
  self.assertTrue(watch.update('a',4.,dict(lead=pose(z=.5)))['rough'])

 def test_speed_changes_do_not_lift_canal_or_clearance_limits(self):
  for clearance,canal in [(1.,False),(None,False),(30.,True)]:
   self.assertEqual(target_speed(True,canal,clearance,pending=True,rough=True),4.)
  self.assertEqual(target_speed(True,False,30,pending=True),10.5)

 def test_calm_hold_excludes_window_peak_and_duplicate_frames(self):
  watch=RoughWaterWatch();watch.update('a',0.,dict(lead=pose(tilt=21.)))
  for t in range(1,54):self.assertTrue(watch.update('a',float(t),dict(lead=pose()))['rough'])
  for _ in range(100):self.assertTrue(watch.update('a',53.,dict(lead=pose()))['rough'])
  self.assertFalse(watch.update('a',54.,dict(lead=pose()))['rough'])

 def test_pause_missing_frames_restart_and_reversed_clock_cannot_count_as_calm(self):
  for reset in ['pause','gap','nonce','reverse']:
   with self.subTest(reset=reset):
    watch=RoughWaterWatch(rough=True)
    for t in range(41):watch.update('a',float(t),dict(lead=pose()))
    nonce='b' if reset=='nonce' else 'a';t=0. if reset=='reverse' else 43.
    if reset=='pause':watch.update(nonce,41.,{},paused=True);t=42.
    self.assertTrue(watch.update(nonce,t,dict(lead=pose()))['rough'])
    for i in range(1,49):self.assertTrue(watch.update(nonce,t+i,dict(lead=pose()))['rough'])
    self.assertFalse(watch.update(nonce,t+49,dict(lead=pose()))['rough'])

 def test_new_wave_peak_resets_calm_hold(self):
  watch=RoughWaterWatch(rough=True)
  for t in range(30):watch.update('a',float(t),dict(lead=pose()))
  watch.update('a',30.,dict(lead=pose(tilt=25.)))
  for t in range(31,84):self.assertTrue(watch.update('a',float(t),dict(lead=pose()))['rough'])
  self.assertFalse(watch.update('a',84.,dict(lead=pose()))['rough'])

 def test_yaw_and_inactive_hull_do_not_create_fake_roughness(self):
  watch=RoughWaterWatch();p=pose();p['orientation']=dict(x=0.,y=0.,z=1.,w=0.)
  self.assertFalse(watch.update('a',0.,dict(lead=p))['rough'])
  watch.update('a',1.,dict(lead=p,follower=pose()))
  watch.update('a',2.,dict(lead=p));self.assertNotIn('follower',watch.samples)


class Arrival(unittest.TestCase):

 def test_wall_time_accounts_for_actual_realtime_factor(self):
  self.assertEqual(estimate_seconds(18000,10.,10.5,1.,'sailing',0),1800.)
  self.assertEqual(estimate_seconds(18000,10.,10.5,.5,'sailing',0),3600.)

 def test_new_cap_is_applied_before_the_physical_hull_slows(self):
  self.assertEqual(estimate_seconds(25200,10.5,7.,1.,'sailing',0),3600.)
  self.assertEqual(estimate_seconds(14400,4.,7.,1.,'sailing',0),3600.)

 def test_missing_stale_or_stopped_motion_has_no_fictional_arrival(self):
  base=dict(distance_m=18000.,speed_mps=10.,command_mps=10.5,actual_rate=1.,status='sailing',freshness=0.)
  invalid=dict(distance_m=[None,-1.,math.inf,math.nan],speed_mps=[None,0.,math.nan],
   command_mps=[0.,math.nan],actual_rate=[None,0.,math.inf],
   status=['paused','recovering'],freshness=[None,11.,math.nan])
  for key,values in invalid.items():
   for value in values:
    with self.subTest(key=key,value=value):self.assertIsNone(estimate_seconds(**dict(base,**{key:value})))



if __name__=='__main__':unittest.main()
