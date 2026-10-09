"""Bounded native escape planning and progress watchdogs for the public voyage.

Escape is exceptional: the starting hull may already touch a coastal cell.
Every sampled footprint must improve or retain clearance, the centre must
remain in measured water, and the endpoint must pass the ordinary buffer.
No pose changes or fabricated route progress occur here.
"""
import math
from collections import deque


class ClockProgressWatch:
    """Catch a nearly frozen engine even when occasional fresh messages arrive.

    Brief insertions and ordinary slow hardware do not cause a restart. Only
    a full minute below one tenth of real time is considered a failed engine.
    Pauses and replacement worlds begin a new observation window.
    """
    def __init__(self):
        self.nonce=None
        self.samples=deque()

    def update(self,nonce,sim_time,wall_time,paused=False):
        if sim_time is None or not math.isfinite(sim_time):return False
        if paused or nonce!=self.nonce or (self.samples and sim_time<self.samples[-1][1]):
            self.nonce=nonce;self.samples.clear()
        if paused:return False
        self.samples.append((wall_time,sim_time))
        while len(self.samples)>1 and wall_time-self.samples[1][0]>=60:
            self.samples.popleft()
        elapsed=wall_time-self.samples[0][0]
        return elapsed>=60 and (sim_time-self.samples[0][1])/elapsed<.1


def escape_route(field, start, heading=0., radius=7.5):
    initial=field.maximum(*start[:2],3.5)
    if initial is None:raise RuntimeError('recovery_terrain_unconfirmed')
    candidates=[]
    for angle in range(0,360,15):
        theta=math.radians(angle)
        for distance in (40.,80.,120.):
            goal=[start[0]+distance*math.cos(theta),start[1]+distance*math.sin(theta),0.]
            end=field.maximum(*goal[:2],radius+35.)
            if end is None or end>-1.5:continue
            previous=initial;valid=True
            for k in range(1,int(distance*2)+1):
                t=k/(distance*2);xy=[a+(b-a)*t for a,b in zip(start[:2],goal[:2])]
                centre=field.maximum(*xy,.5)
                floor=field.maximum(*xy,3.75)
                # The extra .25 m covers the swept gap between samples.
                if centre is None or centre>-.75 or floor is None or floor>initial+1.:
                    valid=False;break
                if floor>previous+.3:
                    valid=False;break
                previous=floor
            if valid:
                turn=abs(math.remainder(theta-heading,2*math.pi))
                candidates.append((distance+turn*5.,goal))
    if not candidates:raise RuntimeError('recovery_no_water_escape')
    return min(candidates,key=lambda p:p[0])[1],bool(initial>-1.5)


class ProgressWatch:
    """Use native simulation time, independent of requested speed or wall lag."""
    def __init__(self):self.anchor=None
    def update(self, nonce, sim_time, position, commanded, recovering=False):
        if self.anchor is None or self.anchor[0]!=nonce or sim_time<self.anchor[1]:
            self.anchor=(nonce,sim_time,list(position));return False
        elapsed=sim_time-self.anchor[1]
        distance=math.dist(position,self.anchor[2])
        if distance>12. or commanded<=.1:
            self.anchor=(nonce,sim_time,list(position));return False
        if elapsed>(90. if recovering else 120.):
            self.anchor=(nonce,sim_time,list(position));return True
        return False


class RouteProgressWatch:
    """Circling is motion, but does not count as progress to a waypoint."""
    def __init__(self):self.anchor=None
    def update(self,nonce,sim_time,plan,pilot,commanded,position=None):
        index=max(0,min(len(plan)-1,int(pilot.get('waypoint',0)))) if plan else 0
        key=(nonce,tuple(tuple(p[:2]) for p in plan),index)
        distance=pilot.get('distance_m');finished=pilot.get('finished')
        if position is not None:
            if not plan:self.anchor=None;return False
            # The command heartbeat and pilot report are asynchronous. The
            # previous path's small distance must not become the new path's
            # best distance, causing a retreat despite real forward progress.
            distance=math.dist(position[:2],plan[index][:2])
        if distance is None or not math.isfinite(distance) or commanded<=.1 or finished:
            self.anchor=None;return False
        if self.anchor is None or self.anchor[0]!=key or sim_time<self.anchor[1]:
            self.anchor=(key,sim_time,distance);return False
        if distance<self.anchor[2]-12.:
            self.anchor=(key,sim_time,distance);return False
        if sim_time-self.anchor[1]>90.:
            self.anchor=(key,sim_time,distance);return True
        return False


def coastal_frontiers(start,goal,visited):
    """Explore a bay's distant water instead of oscillating along its beach.

    The caller still verifies every endpoint and entire swept hull corridor.
    A modest backward detour is necessary when a coarse global waypoint lies
    across land. Previously sailed endpoints are penalized, never obstacles.
    """
    direction=math.atan2(goal[1]-start[1],goal[0]-start[0]);candidates=[]
    offsets=[0]+[sign*angle for angle in range(15,180,15) for sign in (1,-1)]+[180]
    for offset in offsets:
        for distance in (1200.,900.,600.,400.,200.,100.):
            theta=direction+math.radians(offset)
            p=[start[0]+distance*math.cos(theta),start[1]+distance*math.sin(theta),0.]
            novelty=min((math.dist(p[:2],v[:2]) for v in visited),default=10000.)
            progress=(p[0]-start[0])*math.cos(direction)+(p[1]-start[1])*math.sin(direction)
            score=progress-2000.*max(0.,1-novelty/350.)
            # Short goals at a dead-end beach repeatedly reset steering. Use
            # them only if no confirmed distant corridor is available.
            candidates.append((distance>=400.,score,p))
    return [p for _,_,p in sorted(candidates,key=lambda v:(v[0],v[1]),reverse=True)]
