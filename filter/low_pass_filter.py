## low pass filter
from math import pi



class LPF:

    whoami = "LowPassFilter"

    def __init__(self, band_limit, dt):
        self.w  = band_limit
        self.dt = dt

        self.prev_data = 9.8


    def _filter(self, data):
        ## filtering
        out = 0
        out += data
        out -= self.prev_data
        out *= self.w
        out *= self.dt
        out += self.prev_data

        self.prev_data = out

        return out