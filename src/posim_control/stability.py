"""Attitude safety for the private voyage; never changes participant dynamics."""
import math


def tilt_degrees(q):
    values = [float(q[k]) for k in ('x', 'y', 'z', 'w')]
    norm = sum(v*v for v in values)
    if not all(math.isfinite(v) for v in values) or not .95 < norm < 1.05:
        raise ValueError('native_orientation_invalid')
    x, y, _, _ = values
    return math.degrees(math.acos(max(-1., min(1., 1.-2.*(x*x+y*y)/norm))))


class CapsizeWatch:
    """Require sustained measured tilt; wave crests and duplicate frames don't count."""
    def __init__(self):
        self.samples = {}

    def update(self, name, nonce, sim_time, orientation, paused=False):
        if paused:
            self.samples.pop(name, None)
            return False
        angle = tilt_degrees(orientation)
        previous = self.samples.get(name)
        if angle < 80.:
            self.samples.pop(name, None)
            return False
        if previous is None or previous[0] != nonce or sim_time < previous[2]:
            self.samples[name] = (nonce, sim_time, sim_time)
            return False
        self.samples[name] = (nonce, previous[1], sim_time)
        return sim_time-previous[1] >= 2.


def restore_attitude(native, heading_deg):
    """Restore at the SAME geographic checkpoint, never inherit a capsized pose.

    An audited world recovery resets only unsafe tilt/heave and momentum. Normal
    restarts retain their measured roll, pitch, height and body velocities.
    """
    q = native['local_pose']['orientation']
    tilt = tilt_degrees(q)
    yaw = math.radians(90.-heading_deg)
    if tilt >= 60.:
        return dict(angles=[0., 0., yaw], z=.3, linear=[0., 0., 0.],
                    angular=[0., 0., 0.], righted=True, previous_tilt_deg=tilt)
    norm = math.sqrt(sum(q[k]**2 for k in ('x', 'y', 'z', 'w')))
    x, y, z, w = (q[k]/norm for k in ('x', 'y', 'z', 'w'))
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
    velocity = native['velocity']
    return dict(angles=[roll, pitch, yaw], z=native['local_pose']['position']['z'],
                linear=[velocity['linear_body'][k] for k in ('x', 'y', 'z')],
                angular=[velocity['angular_body'][k] for k in ('x', 'y', 'z')],
                righted=False, previous_tilt_deg=tilt)
