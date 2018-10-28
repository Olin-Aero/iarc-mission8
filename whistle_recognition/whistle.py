#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Adapted from audiolazy_example.py, which came with this license...
#
# This file is part of AudioLazy, the signal processing Python package.
# Copyright (C) 2012-2016 Danilo de Jesus da Silva Bellini
#
# AudioLazy is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, version 3 of the License.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
whistle detector.  Work in progress!
"""
# from __future__ import division
from audiolazy import sHz, chunks, AudioIO, line, pi, window
import numpy as np
import collections, sys, threading, time

# AudioLazy init
rate = 44100
s, Hz = sHz(rate) # s = rate, Hz = tau / rate

length = 2 ** 14
data = collections.deque([0.] * length, maxlen=length)
wnd = np.array(window.hamming(length)) # For FFT

searchFreq = 100 # Hz; minimum time between checking for whistles.

minAmp = 0.0001 # Minimum absolute amplitude of a whistle
minSharp = 0.0001 # Minimum "sharpness" of a whistle

maxVariance = 0.001 # Maximum variance in frequency for a whistle
tonicResetlen = 2 # Seconds to reset the scale

relativeNotes = np.log([1/2, 5/8, 3/4, 1, 5/4, 3/2, 2])

api = sys.argv[1] if sys.argv[1:] else None # Choose API via command-line
chunks.size = 1 if api == "jack" else 16

# Creates a data updater callback
def update_data():
  with AudioIO(api=api) as rec:
    for el in rec.record(rate=rate): # Appears to run at about 44100 hz
      data.append(el)
      if update_data.finish:
        break

# Creates the data updater thread
update_data.finish = False
th = threading.Thread(target=update_data)
th.start() # Actually start updating data

# Set up some variables preserved between search iterations
whistle = [] # This whistle so far
whistleLen = 0 # The whistle's length, in seconds
lastTime = time.clock()
tonic = 0 # Freq, in Hz, of the tonic note off of which other whistles will be judged

try:
  while True:
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
      print("Freq: " + str(freq) + "\tAmplitude: " + str(amplitude) + "\tSharpness: " + str(sharpness))
      whistle += [freq]
      whistleLen += timeSinceLast
      whistleAvg = np.exp(np.mean(np.log(whistle))) # Take the log mean to find the cental frequency
      if tonicResetlen <= whistleLen:
        if np.var(np.log(whistle)) < maxVariance:
          tonic = whistleAvg
          print("Reset tonic to " + str(tonic) + " Hz")
        else:
          print("Variance too high")
      else:
        if tonic == 0:
          print("Tonic not set")
        else:
          logDist = np.log(whistleAvg) - np.log(tonic)
          print("Log distance: " + str(logDist))
          # Find the closest note
          closestNote = -1
          closestDist = np.Inf
          for i, n in enumerate(relativeNotes):
            if abs(logDist - n) < closestDist:
              closestDist = logDist - n
              closestNote = i
          print("Closest note index: " + str(closestNote))
    else:
      whistle = []
      whistleLen = 0
 
    
    #Try to keep times as close to searchFreq as possible
    if timeSinceLast < (1 / searchFreq):
      time.sleep((1 / searchFreq) - timeSinceLast)
except KeyboardInterrupt:
  pass

# Stop the recording thread after closing the window
update_data.finish = True
th.join()
