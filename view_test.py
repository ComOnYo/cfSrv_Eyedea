from __future__ import print_function
from threading import Thread
import signal
import time
import sys
import zmq
import os

log_client = [0]*10
log_thread = [0]*10

def LogServer(idx="") :
    return "tcp://localhost:20"+idx+"1"

class _LogThread(Thread):
    def __init__(self, socket, *args):
        super(_LogThread, self).__init__(*args)
        self._socket = socket
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.thrust = 0
        self.bat = 0.0
        self.bat_state = 0.0
        self.zw = 0.0
    def run(self):
        while True:
            log = self._socket.recv_json()
            if log["event"] == "data":
                self.roll = log["variables"]["stabilizer.roll"]
                self.pitch = log["variables"]["stabilizer.pitch"]
                self.yaw = log["variables"]["stabilizer.yaw"]
                self.thrust = log["variables"]["stabilizer.thrust"]
                self.bat = log["variables"]["pm.vbat"]
                self.bat_state = log["variables"]["pm.state"]
                print(log["variables"]["acc.zw"])


signal.signal(signal.SIGINT, signal.SIG_DFL)

context = zmq.Context()

for i in range(1):
    #Create socket
    log_client[i] = context.socket(zmq.SUB)
    log_client[i].connect(LogServer(str(i)))
    log_client[i].setsockopt_string(zmq.SUBSCRIBE, u"")

    #Create thread
    log_thread[i] = _LogThread(log_client[i])
    log_thread[i].start()

'''
while True:
    if log_thread[0].pitch is not 0:
        print(" =>pitch : {}".format(log_thread[0].pitch))
    print(log_thread[0].zw)
    #os.system('clear')
    time.sleep(0.01)
'''
