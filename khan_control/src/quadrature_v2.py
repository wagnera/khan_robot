#!/usr/bin/python
# Object for decoding/estimating Quadrature decodings.
# Framework by: Jason Ziglar <jpz@vt.edu>

from collections import deque
from math import pi, floor

class QuadratureEstimator:
  def __init__(self, ticks_per_revolution = 1000.0 / 3):
    self._radians_per_tick = (2.0 * pi) / ticks_per_revolution
    self._position = 0
    self._velocity = 0
    self._a_prev = None
    self._b_prev = None
    self._prev_time = None
    # This is a circular buffer - an array that, when full, drops the oldest
    # entry when a new one is added
    self._times = deque(maxlen = 5)
    # Keys are previous (a, b), then current (a, b). clock[previous][current] returns
    # None if an invalid move, or relative motion otherwise
    self._decode_table = {
    (False, False) : {(False, False) : 0, (False, True) : 1, (True, True) : None, (True, False) : -1},
    (False, True) : {(False, False) : -1, (False, True) : 0, (True, True): 1, (True, False): None},
    (True, True) : {(False, False) : None, (False, True) : -1, (True, True): 0, (True, False): 1},
    (True, False) : {(False, False) : 1, (False, True) : None, (True, True) : -1, (True, False) : 0}
    }
  def update(self, a_state, b_state, time):
    # Updates the state of the encoder based on the inputs given. a_state and
    # b_state need to be resolvable to boolean values representing the A and B
    # channels, while time should be a floating point value representing time.
    # Convert to booleans to handle variants in input more robustly.
    a_state = bool(a_state)
    b_state = bool(b_state)
    # Bail if the old states have not been initialized
    if self._a_prev is None or self._b_prev is None:
      self._save_state(a_state, b_state, time)
      return

    # Create tuples of states for keys
    prev_key = (self._a_prev, self._b_prev)
    curr_key = (a_state, b_state)
    # Lookup offset
    offset = self._decode_table[prev_key][curr_key]

    # Check if offset is not an error
    if offset is not None:
      # Update position
      theta = self._position + (offset * self._radians_per_tick)
      # Store position, wrapped as [-pi, pi] for ease of use
      self._position = theta - (2.0 * pi) * floor((theta + pi) / (2.0 * pi))
      # Add delta-t to circular buffer
      self._times.append((offset, time - self._prev_time))

      # Sum up position offset and how long those updates have taken
      counter = 0
      avg_time = 0
      # For every velocity sample in the circular buffer
      for ii, jj in self._times:
        counter = counter + ii
        avg_time = avg_time + jj

      # Compute velocity, but can't divide by zero (or negative time)
      if avg_time >= 1.0e-3:
        self._velocity = (counter * self._radians_per_tick) / avg_time
      else:
        self._velocity = 0

    # Save state before ending the update
    self._save_state(a_state, b_state, time)
    # Return flag indicating the offset
    return offset

  def _save_state(self, a_state, b_state, time):
    self._a_prev = a_state
    self._b_prev = b_state
    self._prev_time = time

  # Provides accessor for ticks per revolution
  @property
  def ticks_per_revolution(self):
      return self._radians_per_tick / (2 * pi)
  # Provides access to the position, in radians, with no wraparound
  @property
  def position(self):
      return self._position
  @property
  def velocity(self):
      return self._velocity