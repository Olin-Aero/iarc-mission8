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

# AudioLazy init
rate = 44100
s, Hz = sHz(rate) # s = rate, Hz = tau / rate

length = 2 ** 14
data = collections.deque([0.] * length, maxlen=length)
wnd = np.array(window.hamming(length)) # For FFT

searchFreq = 100 # Hz; minimum time between checking for whistles.

minAmp = 0.0001 # Minimum absolute amplitude of a whistle
minSharp = 0.0001 # Minimum "sharpness" of a whistle

maxSlope = 0.02 # Log hz per second; beyond this it's considered two separate whistles
slopeAvgLen = 5 # Number of samples to average the slope
minWhistleLen = 0.3 # seconds; below this, whistles are not processed.  Expect strange behavior if == 0.

maxVariance = 0.5 # Maximum variance in frequency for a whistle
tonicResetLen = 2 # Seconds to reset the scale

maxGroupSpacing = 3; # Seconds between whistles for a group

noteNames = ["tonic", "3rd", "5th", "octave"]
relativeNotes = np.log([1, 5/4, 3/2, 2])

api = sys.argv[1] if sys.argv[1:] else None # Choose API via command-line
chunks.size = 1 if api == "jack" else 16


def groupToCmd(group):
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


def freqToNote(freq, tonic):
  if tonic == 0:
    print("Tonic not set")
    return None

  logDist = np.log(whistleAvg) - np.log(tonic)
  print("Log distance: " + str(logDist))

  # Find the closest note, by difference of log-distances
  closestNote = min(enumerate(relativeNotes), key = lambda x: abs(logDist - x[1]))[0]
  print("Closest note: " + noteNames[closestNote])
  return closestNote


# TODO: Maybe break into an object
lastWhistleTime = None
group = []
def processWhistle(whistle, whistleLen, tonic):
  global lastWhistleTime, group # I am sorry.

  note = freqToNote(whistle, tonic)
  if note is not None:
    # Check timing, maybe start a new group
    now = time.clock()
    if lastWhistleTime is None or (now - lastWhistleTime) > maxGroupSpacing:
      group = [note]
      print("Starting new whistle group (timed out)")
    else:
      group += [note]
    lastWhistleTime = now

    if len(group) == 3:
      cmd = groupToCmd(group)
      if cmd:
        rospy.loginfo("Sent voice command: \"" + cmd + "\"")
        pub.publish(cmd) 
        group = []
      else:
        print("Failed to parse whistles.")

 

# Creates a data updater callback for AudioLazy
def update_data():
  with AudioIO(api=api) as rec:
    for el in rec.record(rate=rate): # Appears to run at about 44100 hz
      data.append(el)
      if update_data.finish:
        break

# Define the rospy publisher
pub = rospy.Publisher("/voice", String, queue_size = 10)

# Start the ROS node
rospy.init_node("whistle_detector", anonymous = True)


# Creates the data updater thread
update_data.finish = False
th = threading.Thread(target=update_data)
th.daemon = True # Don't keep the program alive if this is the only thread
th.start() # Actually start updating data

# Set up some variables preserved between search iterations
whistle = [] # This whistle so far
logWhistle = []
whistleLen = 0 # The whistle's length, in seconds
lastTime = time.clock()
tonic = 0 # Freq, in Hz, of the tonic note off of which other whistles will be judged

try: # Catch ctrl-c nicely.  May no longer be needed with rospy.
  while not rospy.is_shutdown():
    array_data = np.array(data)
    spectrum = np.abs(np.fft.rfft(array_data * wnd)) / length

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

    if minAmp < amplitude and minSharp < sharpness:
      whistle += [freq]
      logWhistle += [np.log(freq)]
      whistleLen += timeSinceLast
      whistleAvg = np.exp(np.mean(logWhistle)) # Take the log mean to find the cental frequency
      if tonicResetLen <= whistleLen:
        if np.var(logWhistle) < maxVariance:
          tonic = whistleAvg
          print("Reset tonic to " + str(tonic) + " Hz")
        else:
          print("Variance too high")
      else:
        if slopeAvgLen+1 <= len(whistle):
          diffs = np.subtract(logWhistle[-slopeAvgLen:], logWhistle[-slopeAvgLen-1:-1])
          if np.sqrt(np.mean(np.square(diffs))) <= maxSlope:
            pass
          else:
            if minWhistleLen <= whistleLen < tonicResetLen:
              processWhistle(whistle, whistleLen, tonic)
            whistle = []
            logWhistle = []
            whistleLen = 0
    else: # if amplitude or sharpness is too low
      if minWhistleLen <= whistleLen < tonicResetLen:
        processWhistle(whistle, whistleLen, tonic)
      whistle = []
      logWhistle = []
      whistleLen = 0
    
    #Try to keep times as close to searchFreq as possible
    if timeSinceLast < (1 / searchFreq):
      time.sleep((1 / searchFreq) - timeSinceLast)
except KeyboardInterrupt:
  pass

# Stop the recording thread
update_data.finish = True
th.join()
