import numpy as np

import time



def guidance(cf, commander):
    
    cmd = np.array([1,1,1])

    for i in range(1,50):
        cf.command = cmd * i

        time.sleep(0.25)

        if i == 10:
            commander._init()

        if i == 40:
            commander._stop()