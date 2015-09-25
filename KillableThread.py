#Stephen Lu, Sept. 2nd 2015

import threading

#Killable thread using a signal handler and Ctrl-C
class KillableThread(threading.Thread):
    def __init__(self, function):
        #function is the function that you wish to run on repeat
        threading.Thread.__init__(self)
        self.func = function
        self.kill_me_please = False

    def run(self):
        
        while not self.kill_me_please:
            self.func()
        return

#######EXAMPLE#######
##Create the function you want to run on loop. Remember, things that only
##need to run once should be outside of this function and in main.
##
##

import signal
import time
import sys

threads = []

def example():
   print "This is running"

def example2():
    print "this is also running"

def signal_handler(sig,frame):
    for thread in threads:
        thread.kill_me_please = True
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

thread1 = KillableThread(example)
thread2 = KillableThread(example2)
threads.append(thread1)
threads.append(thread2)
thread1.start()
thread2.start()

while True:
    time.sleep(1)

