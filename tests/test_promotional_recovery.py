"""Synthetic recovery checks, separate from actual native voyage evidence."""
import math,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
from posim_control.recovery import ProgressWatch,RouteProgressWatch,ClockProgressWatch,escape_route,coastal_frontiers

class Slope:
 def maximum(self,x,y,r):
  return -(x-r)*2-4 if abs(y)+r<250 and abs(x)+r<250 else None

class RecoveryContracts(unittest.TestCase):
 def test_occasional_fresh_messages_do_not_hide_frozen_physics(self):
  watch=ClockProgressWatch()
  for t in range(60):self.assertFalse(watch.update('a',t*.005,float(t)))
  self.assertTrue(watch.update('a',.3,60.))
 def test_clock_watch_tolerates_brief_stalls_pauses_and_normal_slow_hardware(self):
  for ratio in (.11,.5,1.):
   watch=ClockProgressWatch()
   for t in range(180):self.assertFalse(watch.update('a',t*ratio,float(t)))
  watch=ClockProgressWatch()
  for t in range(30):self.assertFalse(watch.update('a',0.,float(t)))
  for t in range(30,100):self.assertFalse(watch.update('a',float(t-30),float(t)))
  self.assertFalse(watch.update('a',70.,200.,True))
  self.assertFalse(watch.update('a',70.,201.))
  self.assertFalse(watch.update('b',0.,301.))
  self.assertFalse(watch.update('b',None,302.))
 def test_preexisting_grounding_retreats_toward_deeper_water(self):
  goal,assisted=escape_route(Slope(),[0.,0.,0.],math.pi)
  self.assertTrue(assisted);self.assertGreater(goal[0],30)
  self.assertLess(Slope().maximum(*goal[:2],19.5),-1.5)
 def test_safe_hull_near_coast_uses_propulsion_only(self):
  goal,assisted=escape_route(Slope(),[4.,0.,0.])
  self.assertFalse(assisted);self.assertGreater(goal[0],4.)
 def test_unknown_or_entire_land_is_never_a_rescue_corridor(self):
  for value in (None,3.):
   class Closed:
    def maximum(self,*args):return value
   with self.assertRaises(RuntimeError):escape_route(Closed(),[0,0,0])
 def test_escape_cannot_cross_a_new_ridge(self):
  class Ridge(Slope):
   def maximum(self,x,y,r):
    if x-r<10<x+r:return 5.
    return super().maximum(x,y,r)
  with self.assertRaises(RuntimeError):escape_route(Ridge(),[0,0,0])
 def test_watch_uses_native_time_and_accepts_world_restart(self):
  w=ProgressWatch();self.assertFalse(w.update('a',1,[0,0],1.))
  for _ in range(100):self.assertFalse(w.update('a',1,[0,0],1.))
  self.assertTrue(w.update('a',123,[0,0],1.))
  self.assertFalse(w.update('b',0,[0,0],1.))
  self.assertFalse(w.update('b',80,[20,0],1.))
  self.assertFalse(w.update('b',180,[22,0],1.))
 def test_idle_is_not_stuck_and_retreat_retries_earlier(self):
  w=ProgressWatch();w.update('a',0,[0,0],0)
  self.assertFalse(w.update('a',1000,[0,0],0))
  self.assertFalse(w.update('a',1001,[0,0],.8,True))
  self.assertTrue(w.update('a',1092,[0,0],.8,True))
 def test_circling_without_waypoint_progress_triggers_recovery(self):
  w=RouteProgressWatch();plan=[[100,0,0]]
  for t in range(90):self.assertFalse(w.update('a',t,plan,dict(waypoint=0,distance_m=100+math.sin(t)*4,finished=False),3))
  self.assertTrue(w.update('a',91,plan,dict(waypoint=0,distance_m=99,finished=False),3))
 def test_waypoint_progress_and_completed_plan_do_not_false_alarm(self):
  w=RouteProgressWatch();plan=[[100,0,0],[200,0,0]]
  for t in range(180):self.assertFalse(w.update('a',t,plan,dict(waypoint=t//100,distance_m=100-t%100,finished=False),3))
  self.assertFalse(w.update('a',1000,plan,dict(waypoint=1,distance_m=4,finished=True),3))
 def test_delayed_previous_plan_distance_does_not_trigger_false_retreat(self):
  w=RouteProgressWatch();plan=[[200,0,0]]
  # A just-issued corridor still sees one report from the old, nearly
  # completed path. Pose belongs to the current measured simulation tick.
  self.assertFalse(w.update('a',0,plan,dict(waypoint=0,distance_m=5,finished=False),1,[0,0]))
  for t in range(1,150):
   self.assertFalse(w.update('a',t,plan,dict(waypoint=0,distance_m=200-t,finished=False),1,[t,0]))
 def test_measured_pose_still_detects_real_circling_and_empty_plan_is_idle(self):
  w=RouteProgressWatch();plan=[[200,0,0]]
  for t in range(90):
   self.assertFalse(w.update('a',t,plan,dict(waypoint=0,distance_m=200-t,finished=False),1,[math.sin(t),0]))
  self.assertTrue(w.update('a',91,plan,dict(waypoint=0,distance_m=1,finished=False),1,[0,0]))
  self.assertFalse(w.update('a',1000,[],dict(distance_m=200),1,[0,0]))
 def test_coastal_frontier_can_leave_dead_end_bay_without_returning_to_track(self):
  start=[0,0,0];goal=[-1000,-1000,0];visited=[[-600,0],[0,-600]]
  candidates=coastal_frontiers(start,goal,visited)
  distant=[p for p in candidates if math.dist(start,p)>=399.99]
  self.assertTrue(distant);self.assertTrue(all(math.dist(start,p)>=399.99 for p in candidates[:len(distant)]))
  # Two equally long corridors: the unexplored direction is ranked first.
  fresh=next(i for i,p in enumerate(candidates) if math.dist(p[:2],[600,0])<1e-5)
  old=next(i for i,p in enumerate(candidates) if math.dist(p[:2],[-600,0])<1e-5)
  self.assertLess(fresh,old)

if __name__=='__main__':unittest.main(verbosity=2)
