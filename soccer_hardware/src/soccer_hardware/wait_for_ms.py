import time
import numpy as np


class WaitForMs:
    """
    Delays using time.sleep() have a tendency to overshoot the target wait time.
    This class uses feedback to adjust the wait time to achieve the target.
    """

    def __init__(self, ms):
        self._t_ref = ms / 1000.0
        self._t_err = 0
        self._e_gain = 0.0
        self._e_lim_high = 0.0
        self._e_lim_low = 0.0
        self._debug = False

    def set_e_gain(self, e):
        """
        Sets the error amplification factor.
        """
        self._e_gain = e

    def set_e_lim(self, high, low):
        """
        Sets the max and min error bounds. Arguments in ms.
        """
        self._e_lim_high = high / 1000.0
        self._e_lim_low = low / 1000.0

    def _limit(self, err):
        """
        Bounds the error feedback, if necessary, so that
        _e_lim_high >= err >= _e_lim_low.
        """
        if err > self._e_lim_high:
            return self._e_lim_high
        elif err < self._e_lim_low:
            return self._e_lim_low
        else:
            return err

    def wait(self):
        """
        Wait with feedback and limiting. Behaves like time.sleep if the gain and
        limits are not set.
        """
        ts = time.time()

        # Adjust wait time based on previous error
        t_wait = self._t_ref + self._t_err
        time.sleep(t_wait)

        # Update error term
        t_waited = time.time() - ts
        err = self._t_ref - t_waited
        self._t_err = self._t_err + self._e_gain * err
        self._t_err = self._limit(err)

        if self._debug:
            print(
                np.round(t_wait * 1000, 4), np.round(t_waited * 1000, 4),
                np.round(err * 1000, 4), np.round(self._t_err * 1000, 4)
            )