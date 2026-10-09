"""Wall-clock arrival estimate from planned route and measured native progress."""
import math


def estimate_seconds(distance_m,speed_mps,command_mps,actual_rate,status,freshness):
 if status!='sailing' or freshness is None or not math.isfinite(freshness) or freshness>10:return None
 values=(distance_m,speed_mps,actual_rate)
 if any(v is None or not math.isfinite(v) for v in values):return None
 if distance_m<0 or speed_mps<.5 or actual_rate<=0:return None
 # Reflect a new rough-water / canal cap immediately, before the physical
 # hull has finished slowing. Never raise a low measured progress speed.
 if command_mps is not None:
  if not math.isfinite(command_mps) or command_mps<.5:return None
  speed_mps=min(speed_mps,command_mps)
 return distance_m/(speed_mps*actual_rate)
