class NullDetector(object):
    """ example class for generic Detector() implementation """
    def __init__(self):
        pass
    def __call__(self, img):
        return (0.0, 0.0)

class NullTracker(object):
    """ example class for generic Tracker() implementation """
    def __init__(self):
        pass
    def init(self, img, box):
        """ 
        Initialize the tracker with the input image and the bounding box.

        Returns:
            any state-related metadata required for later tracking 
        """
        return None
    def __call__(self, img, box, meta):
        """
        Arguments:
            img(A(H,W,3)): Input image.
            box(A(4)): [cx,cy,w,h] encoded box
            meta(?): Extra information field to maintain tracking state.
        Returns:
            box(A(4)): [cx,cy,w,h] new encoded box
            state
        """
        return box, meta


