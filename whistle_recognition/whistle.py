#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Adapted from audiolazy_example.py, which came with this license...
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
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from numpy.fft import rfft
import numpy as np
import collections, sys, threading, time

# AudioLazy init
rate = 44100
s, Hz = sHz(rate) # s = rate, Hz = tau / rate

length = 2 ** 12
data = collections.deque([0.] * length, maxlen=length)
wnd = np.array(window.hamming(length)) # For FFT

histLen = 10 # Number of frames to average

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

spectra = [] # History of spectra
for _ in range(histLen):
  spectra.append([0] * (int(length/2) + 1))

try:
  while True:
    array_data = np.array(data)
    thisSpectrum = np.abs(rfft(array_data * wnd)) / length
    spectra.pop()
    spectra.append(thisSpectrum)
    spectrum = np.mean(spectra, 0) # Average the history

    freqs = np.array(line(length, 0, 2 * pi / Hz).take(length // 2 + 1))
    # The first freq is 0, the last freq value is half of rate, and there are length/2 + 1 of them (so the
    # index of the last is length/2).  So, each freq is about rate/length * its index (+/- 1 ?)

    lower_bound = 100 # Lowest freq (Hz) to look for
    lower_bound_i = int((lower_bound * length) / rate)
    
    spectrum = spectrum[lower_bound_i:]
    freqs = freqs[lower_bound_i:]

    kernel = (-0.1, -0.2, 0.6, -0.2, -0.1)
    d = np.convolve(spectrum, kernel)[2:-2] # trim so that the elements still correspond to the freqs

    maxi = np.argmax(d)
    maxf = freqs[maxi]
    maxr = d[maxi]
    maxa = spectrum[maxi]

    print("Freq: " + str(maxf) + "\tAmplitude: " + str(maxa) + "\tSharpness: " + str(maxr))
 
    time.sleep(0.01)
except KeyboardInterrupt:
  pass

# Stop the recording thread after closing the window
update_data.finish = True
th.join()
