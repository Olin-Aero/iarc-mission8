#!/usr/bin/env python2
"""
Whistle detector.  Publishes whistles, interpreted as voice commands, to a ROS channel.

To use:
  * Set the tonic.  Do this by whistling a steady note for about two seconds, until the terminal displays
    "Reset tonic to X Hz". 
  * Whistle the parts of a "move" command.  Each one has three parts: Drone, direction, and distance.  Each
    part corresponds to a single whistle.  Tone controls the output:
      Tonic -> drone Alpha, direction north, distance 0.5m
      3rd -> drone Bravo, direction east, distance 1m
      Tonic -> drone Charlie, direction south, distance 2m
      Tonic -> drone Delta, direction west, distance 4m

Some parts of this file were adapted from the AudioLazy example "animated_plot.py".
"""
from __future__ import division
from audiolazy import sHz, chunks, AudioIO, line, pi, window
import numpy as np
import collections, sys, threading, time
import rospy
from std_msgs.msg import String

# static configurations
rate = 44100
s, Hz = sHz(rate) # s = rate, Hz = tau / rate
length = 2 ** 14
noteNames = ["tonic", "3rd", "5th", "octave"]
relativeNotes = np.log([1, 5/4, 3/2, 2])
#api = sys.argv[1] if sys.argv[1:] else None # Choose API via command-line
api = None
chunks.size = 1 if api == "jack" else 16

class WhitleDetector(object):
  def __init__(self):
    # Allocating Data
    self.data = collections.deque([0.] * length, maxlen=length)
    self.wnd = np.array(window.hamming(length)) # For FFT
    # TODO: Maybe break into an object
    self.lastWhistleTime = None
    self.group = []
    
    self.th=None
    self.finishedUpdate = False

    # Define the rospy publisher
    self.pub = rospy.Publisher("/voice", String, queue_size = 10)

    # Setting up parameters
    # TODO : define rospy.get_param('~..') for these

    self.searchFreq = 100 # Hz; minimum time between checking for whistles.

    self.minAmp = 0.0001 # Minimum absolute amplitude of a whistle
    self.minSharp = 0.0001 # Minimum "sharpness" of a whistle

    self.maxSlope = 0.02 # Log hz per second; beyond this it's considered two separate whistles
    self.slopeAvgLen = 5 # Number of samples to average the slope
    self.minWhistleLen = 0.3 # seconds; below this, whistles are not processed.  Expect strange behavior if == 0.

    self.maxVariance = 0.5 # Maximum variance in frequency for a whistle
    self.tonicResetLen = 2 # Seconds to reset the scale

    self.maxGroupSpacing = 3 # Seconds between whistles for a group

  def groupToCmd(self, group):
    """
    Tries to convert a series of 3 whistles into a "voice" command.
    Whistles represent drone index, direction, and distance, in that order.
    Returns the command as a string, or None otherwise.
    """

    # Each command has three parts: drone name, direction, distance.
    # These convert the four whistled tones into each of those parts.
    drones = ["alpha", "bravo", "charlie", "delta"]
    dirs = ["north", "east", "south", "west"]
    dists = ["0.5", "1", "2", "4"]

    if len(group) != 3:
      return None
    
    cmd = " ".join([drones[group[0]], dirs[group[1]], dists[group[2]]])
    return cmd

  def freqToNote(self, freq, tonic, whistleAvg):
    if tonic == 0:
      rospy.loginfo("Tonic not set")
      return None

    logDist = np.log(whistleAvg) - np.log(tonic)
    rospy.loginfo("Log distance: " + str(logDist))

    # Find the closest note, by difference of log-distances
    closestNote = min(enumerate(relativeNotes), key = lambda x: abs(logDist - x[1]))[0]
    rospy.loginfo("Closest note: " + noteNames[closestNote])
    return closestNote

  def processWhistle(self, whistle, whistleLen, tonic, whistleAvg):
    note = self.freqToNote(whistle, tonic, whistleAvg)
    cmd = None
    if note is not None:
      # Check timing, maybe start a new group
      now = time.clock()
      # requires fixing the group logics
      if self.lastWhistleTime is None or (now - self.lastWhistleTime) > self.maxGroupSpacing:
        self.group = [note]
        rospy.loginfo("Starting new whistle group (timed out)")
      else:
        self.group += [note]
      self.lastWhistleTime = now
      #return cmd
      #rospy.loginfo('group', self.group)
      print('group', self.group)
      if len(self.group) == 3:
        cmd = self.groupToCmd(self.group)
    return cmd

  # Creates a self.data updater callback for AudioLazy
  def update_data(self):
    with AudioIO(api=api) as rec:
      for el in rec.record(rate=rate): # Appears to run at about 44100 hz
        self.data.append(el)
        if self.finishedUpdate:
          break

  def start(self):
    # Creates the self.data updater thread
    self.finishedUpdate = False
    self.th = threading.Thread(target=self.update_data)
    self.th.daemon = True # Don't keep the program alive if this is the only thread
    self.th.start() # Actually start updating self.data
  
  def stop(self):
    # Stop the recording thread
    self.finishedUpdate = True
    self.th.join()

  def run(self):
    self.start()

    # Set up some variables preserved between search iterations
    whistle = [] # This whistle so far
    logWhistle = []
    whistleLen = 0 # The whistle's length, in seconds
    lastTime = time.clock()
    tonic = 0 # Freq, in Hz, of the tonic note off of which other whistles will be judged

    try: # Catch ctrl-c nicely.  May no longer be needed with rospy.
      while not rospy.is_shutdown():
        #TODO: make sure same self.data is not processed twice
        array_data = np.array(self.data)
        spectrum = np.abs(np.fft.rfft(array_data * self.wnd)) / length

        freqs = np.array(line(length, 0, 2 * pi / Hz).take(length // 2 + 1))
        # The first freq is 0, the last freq value is half of rate, and there are length/2 + 1 of them (so the
        # index of the last is length/2).  So, each freq is about rate/length * its index (+/- 1 ?)

        lower_bound = 400 # Lowest freq (Hz) to look for
        lower_bound_i = int((lower_bound * length) / rate)
        
        spectrum = spectrum[lower_bound_i:]
        freqs = freqs[lower_bound_i:]

        kernel = (-0.1, -0.2, 0.6, -0.2, -0.1)
        d = np.convolve(spectrum, kernel)[2:-2] # trim so that the elements still correspond to the freqs

        maxi = np.argmax(d) 
        freq = freqs[maxi]
        sharpness = d[maxi]
        amplitude = spectrum[maxi]

        # Do timing stuff just before we record the time, to prevent off-by-1ish
        thisTime = time.clock()
        timeSinceLast = thisTime - lastTime
        lastTime = thisTime

        if self.minAmp < amplitude and self.minSharp < sharpness:
          whistle += [freq]
          logWhistle += [np.log(freq)]
          whistleLen += timeSinceLast
          whistleAvg = np.exp(np.mean(logWhistle)) # Take the log mean to find the cental frequency
          if self.tonicResetLen <= whistleLen:
            if np.var(logWhistle) < self.maxVariance:
              tonic = whistleAvg
              rospy.loginfo("Reset tonic to " + str(tonic) + " Hz")
            else:
              rospy.loginfo("Variance too high")
          else:
            if self.slopeAvgLen+1 <= len(whistle):
              diffs = np.subtract(logWhistle[-self.slopeAvgLen:], logWhistle[-self.slopeAvgLen-1:-1])
              if np.sqrt(np.mean(np.square(diffs))) <= self.maxSlope:
                pass
              else:
                if self.minWhistleLen <= whistleLen < self.tonicResetLen:
                  cmd = self.processWhistle(whistle, whistleLen, tonic, whistleAvg)
                  if cmd:
                    #print('cmd', cmd)
                    rospy.loginfo("Sent voice command: \"" + cmd + "\"")
                    self.pub.publish(cmd) 
                    self.group = []
                  else:
                    rospy.logerr("Failed to parse whistles.")
                whistle = []
                logWhistle = []
                whistleLen = 0
        else: # if amplitude or sharpness is too low
          pass
          # TODO: What is this doing here?
          # if self.minWhistleLen <= whistleLen < self.tonicResetLen:
          #   processWhistle(whistle, whistleLen, tonic)
          whistle = []
          logWhistle = []
          whistleLen = 0
        
        #Try to keep times as close to self.searchFreq as possible
        if timeSinceLast < (1 / self.searchFreq):
          time.sleep((1 / self.searchFreq) - timeSinceLast)
    except KeyboardInterrupt:
      pass

    self.stop()

def main():
  # Start the ROS node
  rospy.init_node("whistle_detector", anonymous = True)
  node = WhitleDetector()
  node.run()

if __name__ == "__main__":
  main()