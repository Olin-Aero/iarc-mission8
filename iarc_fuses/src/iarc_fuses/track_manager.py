class TrackState(object):
    """
    Simple Track State Enum.
    """
    TRACK_NULL = 0
    TRACK_GOOD = 1
    TRACK_LOST = 2

class TrackData(object):
    """
    Track data, with the following fields:

    src_(str)     : Observation Source
    cid_(str)     : Classification ID
    img_(array)   : Associated Image Data
    box_(array)   : Associated Bounding Box
    stamp_(float) : Last observed TimeStamp
    meta_(?)      : Metadata for tracking
    """
    def __init__(self, src, cid, img, box, stamp, meta=None):
        self.src_ = src
        self.cid_ = cid
        self.img_ = img
        self.box_ = box
        self.stamp_ = stamp
        self.meta_ = meta

        self.cnt_ = 1 # count the number of frames seen
    def __repr__(self):
        return '[{}]{}-({})'.format(self.src_, self.cid_, self.box_)

class TrackManager(object):
    def __init__(self, tracker):
        self.tracker_ = tracker
        self.track_   = []

        self.min_p_ = 0.5
        self.max_t_ = 10.0

    def match(self, trk, obs, strict=False):
        """
        Attempt match between trk(old) and obs(new).

        Returns:
            match(bool): whether trk and obs belong to the same object.
        """
        if trk.cid_ != obs.cid_:
            return False
        if trk.src_ != obs.src_:
            # TODO : the same object MAY potentially be observed from multiple sources.
            # Currently not taking such scenarios into account.
            return False
        if box_iou(trk.box_, obs.box_) < 0.5: # TODO : magic?
            # filter by jaccard overlap
            return False

        if strict:
            # strict: attempt "track"
            # computationally costly, so avoid this in usual cases.
            res = self.tracker_(obs.img_, trk.box_, trk.meta_)
            box, meta = res
            score = self.tracker_.get_confidence(meta)
            if score < self.min_p_:
                return False
        return True

    def append(self, obs):
        """
        Add new observation to the tracking list.
        obs(list): list of TrackData() representing new detections.

        TODO : enable tracks recovery?
        """
        obs_new = []
        for o in obs:
            for t in self.track_:
                if self.match(t, o):
                    break
            else:
                # no match found
                obs_new.append( o )

        for o in obs_new:
            o.meta = self.tracker_.init(o.img, o.box)
            self.track_.append(o)

    def filter(self, stamp, tracks):
        res = []
        for t in tracks:
            if (t.box_ is None):
                # no box to track
                continue
            if (stamp - t.stamp_) < self.max_t_
                # stale
                continue
            res.append( t )
        return res

    def process(self, src, img, stamp):
        """
        Process incoming image.
        """
        for t in self.track_:
            if (t.src_ != src):
                continue
            res = self.tracker_(img, t.box_, t.meta_)
            box, meta = res
            score = self.tracker_.get_confidence(meta)
            if score > self.min_p_:
                # successful : update track
                t.box_, t.meta_ = box, meta
                t.cnt_ += 1
                t.stamp_ = stamp
        # update tracks
        self.track_ = self.filter(stamp, self.track_)
