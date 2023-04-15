from threading import Thread

import time



class Tutorial(Thread):


    def run(self):
        
        i, = self._args

        while i < 10:

            print(i)

            i += 1

            time.sleep(0.1)

    
    def join(self):
        super().join()

        print('thread finish')





if __name__ == "__main__":

    trl = Tutorial(args=(0,), daemon=True)

    trl.start()

    for i in range(8):
        print('thread start')
        time.sleep(0.1)

    trl.join()