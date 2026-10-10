"""Physical cruise policy; shoreline proximity alone is not a speed limit."""
import math
from collections import deque
from .stability import tilt_degrees

CRUISE_MPS=10.5
ROUGH_MPS=7.
APPROACH_MPS=CRUISE_MPS
FOLLOWER_MAX_MPS=11.9
SHALLOW_MPS=4.
CANAL_MPS=4.

def sensitive_water(near_port,canal,clearance,caution=False):
 return bool(canal or clearance is None or clearance<2 or caution)

def target_speed(near_port,canal,clearance,pending=False,rough=False):
 # Depth constrains safe speed; a port label and background downloads do not.
 if clearance is None:return SHALLOW_MPS
 if clearance<2:return SHALLOW_MPS
 if canal:return CANAL_MPS
 return ROUGH_MPS if rough else CRUISE_MPS


class RoughWaterWatch:
 """Private voyage cruise reduction from both hulls' measured wave motion.

 Keep a low-speed hold until 45 advancing simulation seconds of measured calm.
 Restarts, missing observations and duplicate frames cannot count as calm.
 """
 WINDOW_SECONDS=8.
 CALM_SECONDS=45.
 TILT_ON_DEGREES=30.
 TILT_OFF_DEGREES=10.
 HEAVE_ON_METRES=1.2
 HEAVE_OFF_METRES=.8

 def __init__(self,rough=False):
  self.rough=rough;self.nonce=None;self.last=None;self.calm_since=None;self.samples={}

 def update(self,nonce,sim_time,poses,paused=False):
  if not math.isfinite(sim_time):raise ValueError('rough_water_clock_invalid')
  if paused or nonce!=self.nonce or self.last is not None and (sim_time<self.last or sim_time-self.last>2.):
   self.samples={};self.calm_since=None;self.last=None;self.nonce=nonce
  if paused:return self.snapshot()
  if self.last is not None and sim_time<=self.last:return self.snapshot()
  self.last=sim_time
  self.samples={name:values for name,values in self.samples.items() if name in poses}
  for name,pose in poses.items():
   z=float(pose['position']['z']);tilt=tilt_degrees(pose['orientation'])
   if not math.isfinite(z):raise ValueError('rough_water_pose_invalid')
   values=self.samples.setdefault(name,deque());values.append((sim_time,z,tilt))
   while values and values[0][0]<sim_time-self.WINDOW_SECONDS:values.popleft()
  metrics=self.snapshot()
  if metrics['tilt_peak_deg']>=self.TILT_ON_DEGREES or metrics['heave_range_m']>=self.HEAVE_ON_METRES:
   self.rough=True;self.calm_since=None
  elif metrics['observed_seconds']>=4. and metrics['tilt_peak_deg']<self.TILT_OFF_DEGREES and metrics['heave_range_m']<self.HEAVE_OFF_METRES:
   if self.calm_since is None:self.calm_since=sim_time
   if sim_time-self.calm_since>=self.CALM_SECONDS:self.rough=False
  else:self.calm_since=None
  return self.snapshot()

 def snapshot(self):
  values=list(self.samples.values())
  return dict(rough=self.rough,speed_cap_mps=ROUGH_MPS if self.rough else CRUISE_MPS,
   tilt_peak_deg=max((max(p[2] for p in v) for v in values),default=0.),
   heave_range_m=max((max(p[1] for p in v)-min(p[1] for p in v) for v in values if v[-1][0]-v[0][0]>=4.),default=0.),
   observed_seconds=min((v[-1][0]-v[0][0] for v in values),default=0.))
