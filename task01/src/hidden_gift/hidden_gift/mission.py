"""STUDENT FILE. Implement the search state machine. No ROS calls in this module.
Use raster() and drive_to(); never read simulator internals or web /state.
Return Outcome('absent') ONLY after covering the entire requested area.
Return Outcome('found', x, y) after measuring the rectangle centre.
"""
import math
from dataclasses import replace
from course_lab.world import Area, Sample, Decision, Outcome
from course_lab.navigation import drive_to, raster
from course_lab.probe import RectangleProbe

class Mission:
    def __init__(self, area: Area):
        self.area = area
        self.waypoints = raster(area)
        self.index = 0
        self.probe = None
        self.distance = 0.0
        self.last = None

    def step(self, sample: Sample) -> Decision:
        # TODO M1: follow raster; on green INSIDE area instantiate RectangleProbe.
        # Probe already measures boundaries. Delegate future samples to probe.step().
        # Basic feedback and all safety/failure policy are supplied by the adapter.
        # Starter is intentionally stopped, not a working submission.
        
        # If gift is seen, then probe owns the robot while it measures the centre 
        # *The probe counts its own distance from zero
        if self.probe is not None:
            return self._probe_step(sample)
        if self.last is not None and sample.sequence != self.last.sequence:
            self.distance += math.hypot(sample.x - self.last.x, sample.y - self.last.y)
        self.last = sample
        if sample.green and self.area.contains(sample.x, sample.y):
            self.probe = RectangleProbe(self.area, sample)
            return self._probe_step(sample)

        # Skip every waypoint which already reached
        # 'absent' only after the last one to avoid problems
        while self.index < len(self.waypoints):
            v, w, reached = drive_to(sample, self.waypoints[self.index])
            if not reached:
                return Decision(v, w, distance=self.distance)
            self.index += 1
        return Decision(outcome=Outcome('absent'), distance=self.distance)

    def _probe_step(self, sample: Sample) -> Decision:
        d = self.probe.step(sample)
        return replace(d, distance=self.distance + d.distance)
