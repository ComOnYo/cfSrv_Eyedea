from __future__ import print_function
from threading import Thread
import signal
import time
import sys
import zmq
import os

LOG_SERVER = "tcp://localhost:2001"

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
    def run(self):
        while True:
            log = self._socket.recv_json()
            if log["event"] == "data":
                self.roll = log["variables"]["stabilizer.roll"]
                self.pitch = log["variables"]["stabilizer.pitch"]
                self.yaw = log["variables"]["stabilizer.yaw"]
                self.thrust = log["variables"]["stabilizer.thrust"]
                self.bat = log["variables"]["pm.vbat"]

class prt(Thread):
    def __init__(self, idx, *args):
        super(prt, self).__init__(*args)
        self.idx = idx
    def run(self):
        while True:
            print(str(self.idx+1) + " => roll : {} pitch : {} yaw : {} thrust : {} bat : {}\r".format(log_thread[self.idx].roll, log_thread[self.idx].pitch, log_thread[self.idx].yaw, log_thread[self.idx].thrust, log_thread[self.idx].bat), end = "")

signal.signal(signal.SIGINT, signal.SIG_DFL)

context = zmq.Context()

for i in range(10):
    #Create socket
    log_client[i] = context.socket(zmq.SUB)
    log_client[i].connect(LogServer(str(i)))
    log_client[i].setsockopt_string(zmq.SUBSCRIBE, u"")

    #Create thread
    # Start async threads
    log_thread[i] = _LogThread(log_client[i])
    log_thread[i].start()

#p = prt(0)
#p.start()

while True:
    os.system('clear')
    for i in range(10):
        print(str(i+1) + " => roll : {} pitch : {} yaw : {} thrust : {} bat : {}".format(log_thread[i].roll, log_thread[i].pitch, log_thread[i].yaw, log_thread[i].thrust, log_thread[i].bat))
    #os.system('clear')
    time.sleep(0.5)


sys.exit(1)
print("done!")

"""

client.send_json(connect)
resp = client.recv_json()
if resp["status"] != 0:
    print("fail! {}".format(resp["msg"]))
    sys.exit(1)
print("done!")


log = {
    "version" : 1,
    "cmd" : "log",
    "action" : "create",
    "name" : "Test log block",
    "period" : 30,
    "variables" : [
        "pm.vbat",
        "stabilizer.roll",
        "stabilizer.pitch"
    ]
}

print("Creating logging {} ...".format(log["name"]), end=' ')
client.send_json(log)
resp = client.recv_json()
if resp["status"] == 0:
    print("done!")
else:
    print("fail! {}".format(resp["msg"]))


log_cmd = {
    "version": 1,
    "cmd": "log",
    "action": "start",
    "name": "Test log block"
}
print("Starting logging {} ...".format(log_cmd["name"]), end=' ')
client.send_json(log_cmd)
resp = client.recv_json()
if resp["status"] == 0:
    print("done!")
else:
    print("fail!")
"""
