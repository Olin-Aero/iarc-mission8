from QRCombiner import *
from QRDetector import *

# Tests the interfacing of the QRDetector and QRCombiner
det = Detector()
det.run()	
comb = Combiner()
print(comb.run())