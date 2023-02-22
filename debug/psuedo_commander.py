from threading import Thread

import time



class PsuedoCommander(Thread):

    def __init__(self, cf):
        super().__init__(daemon=True)

        self.cf = cf

        self.ready_for_command = False

    
    def run(self):
        cf = self.cf
        flying = False

        while not self.ready_for_command:
            print('stop')
            time.sleep(0.1)

        while self.ready_for_command:

            if flying:
                try:
                    command = cf.command
                    print('1', command)

                except:
                    command = [0,0,9.81]
                    print('2', command)

            else:
                try:
                    command = cf.command
                    flying  = True
                    print('3', command)

                except:
                    print('4', command)
                    continue

            time.sleep(0.1)

    
    def join(self):
        super().join()

        print('thread end')

        
    def _init(self):
        self.ready_for_command = True

    
    def _stop(self):
        self.ready_for_command = False