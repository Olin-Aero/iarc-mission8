#!/usr/bin/env python2

import cv2
import numpy as np
import utils
import rospy

"""
Time-based filtering of CV bounding boxes.

This uses several assumptions about the world (e.g. that things usually stay in
about the same place, rather than disappearing or teleporting) to filter noisy
CV-derived bounding boxes.  Its primary application is finding humans, but it will
probably work for other things as well.
"""


class Box:
    """
    A box consists of (sides, first_seen, last_seen).
    sides are a 4-list (TODO: Or 4-tuple?) of coords, expressed as
    fractions of the total image size, in the order (upper y, left x, 
    lower y, right x).
    first_ and last_seen are rospy times representing the first and
    last times we've seen that box.
    """

    def __init__(self, sides, first_seen=None, last_seen=None):
        self.sides = sides
        self.first_seen = first_seen
        self.last_seen = last_seen

    def dists(self, other):
        """
        Absolute-value distances between my sides and the sides of the other box.
        """
        # Unfortunately, "other's" is not a valid Python variable name.
        return [abs(mine - others) for (mine, others) in zip(self.sides, other.sides)]

    def num_far_sides(self, other, max_dist):
        return sum([1 if (dist > max_dist) else 0 for dist in self.dists(other)])


class Filter:
    """
    A filter object.  Has internal state.
    """

    def toRosTime(t):
        if type(t) is not rospy.rostime.Time:
            return rospy.Time.from_sec(t)
        else:
            return t

    def __init__(self, min_age=0.01, max_age=0.5, max_side_dist=0.2, max_far_sides=2):
        """
        Parameters
        ----------
        min_age : float or rospy.rostime.Time
          If a bbox is younger than this, it gets filtered out.
        max_age : float or rospy.rostime.Time
          If a bbox has not been seen for this long, it gets filtered out.
        max_side_dist : float
          Maximum distance, as a fraction of bbox diagonal size, between old
          side location and new side location, before the bbox is
          considered new, rather than another sighting of an existing bbox.
        max_far_sides : int
          If more than this many sides are more than max_side_dist from their
          old locations, then the bbox is considered new.
        """

        self.min_age = toRosTime(min_age)
        self.max_age = toRosTime(max_age)

        self.max_side_dist = max_side_dist
        self.max_far_sides = max_far_sides

        self.old_boxes = []

    def __call__(self, now, boxes):
        """
        Executes the filter, incorporating new data and returning known boxes.
        """

        now = toRosTime(now)
        boxes = [Box(box, last_seen=now) for box in boxes]  # convert to Box objects

        # Incorporate new boxes
        # Yes, this is O(n^2).  TODO: Make this faster if it needs it
        new_boxes = []
        for old_box in self.old_boxes:
            for box in boxes:
                if old_box.num_far_sides(box) <= max_far_sides:
                    new_boxes += Box(box.sides, old_box.first_seen, box.last_seen)
        self.old_boxes = new_boxes

        # Delete old boxes
        self.old_boxes = [
            box for box in self.old_boxes if (now - box.last_seen) <= self.max_age
        ]

        # Return valid boxes
        return [
            box
            for box in self.old_boxes
            if (box.last_seen - box.first_seen) >= self.min_age
        ]

    def clear(self):
        """
        Clears the filter's memory, forgetting all bboxes.
        """
        self.old_boxes = []
