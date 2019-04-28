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


class Filter:
    """
  A filter object.  Has internal state.
  """

    def __init__(
        self, min_age=0.01, max_age=0.5, all_corners=False, max_corner_dist=0.2
    ):
        """
        Parameters
        ----------
        min_age : float or rospy.rostime.Time
          If a bbox is younger than this, it gets filtered out.
        max_age : float or rospy.rostime.Time
          If a bbox has not been seen for this long, it gets filtered out.
        all_corners : bool
          If True, bboxes are considered new if any of their corners is too far
          from where it was last.  If False, then three corners have to be too
          far for it to get filtered out.
        max_corner_dist : float
          Maximum distance, as a fraction of bbox diagonal size, between old
          corner location and new corner location, before the bbox is
          considered new, rather than another sighting of an existing bbox.

        """

        # Some args can be float seconds or ros time objects.
        def toRosTime(t):
            if type(t) is not rospy.rostime.Time:
                return rospy.Time.from_sec(t)
            else:
                return t

        self.min_age = toRosTime(min_age)
        self.max_age = toRosTime(max_age)

        self.all_corners = all_corners
        self.max_corner_dist = max_corner_dist

        """
        A box consists of (corners, time).
        Corners are a 4-list (TODO: Or 4-tuple?) of coords, expressed as
        fractions of the total image size, in the order (upper y, left x, 
        lower y, right x).
        Time is the last rospy.rostime.Time that the box was seen.
        """
        self.old_boxes = []
