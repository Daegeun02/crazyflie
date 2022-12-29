## weight moving average filter
from numpy import exp



class EMAF:

    whoiam = 'ExponentialMovingAverageFilter'

    ewf = exp(-1)

    def __init__(self):
        self.prev_data = 9.8

    def _filter(self, data):
        ewf = self.__class__.ewf

        out = ewf * self.prev_data + (1 - ewf) * data

        self.prev_data = out

        return out

    
class MAF:

    whoiam = 'MovingAverageFilter'

    def __init__(self):
        self.n = 0
        self.prev_data = 9.8
