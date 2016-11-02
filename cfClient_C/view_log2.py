from __future__ import print_function
from threading import Thread
import signal
import time
import sys
import zmq

LOG_SERVER = "tcp://localhost:2011"

class _LogThread(Thread):
    def __init__(self, socket, *args):
        super(_LogThread, self).__init__(*args)
        self._socket = socket
    def run(self):
        while True:
            log = self._socket.recv_json()
            if log["event"] == "data":
                print("roll : {}".format(log["variables"]["stabilizer.roll"]))
                print("pitch : {}".format(log["variables"]["stabilizer.pitch"]))
                print("yaw : {}".format(log["variables"]["stabilizer.yaw"]))
                print("thrust : {}".format(log["variables"]["stabilizer.thrust"]))
                print("battery : {}, {}".format(log["variables"]["pm.vbat"], log["variables"]["pm.chargeCurrent"]))
                #print("m1 : {}".format(log["variables"]["motor.m1"]))
                #print("m2 : {}".format(log["variables"]["motor.m2"]))
#print("m3 : {}".format(log["variables"]["motor.m3"]))
#                print("m4 : {}".format(log["variables"]["motor.m4"]))
#                zmqsocket.send_string((str)(log["variables"]["stabilizer.roll"]))
#               message = zmqsocket.recv_string()
#               zmqsocket.send_string((str)(log["variables"]["stabilizer.pitch"]))
#               message = zmqsocket.recv_string()
            if log["event"] == "created":
                print("Created block {}".format(log["name"]))
            if log["event"] == "started":
                print("Started block {}".format(log["name"]))
            if log["event"] == "stopped":
                print("Stopped block {}".format(log["name"]))	    
            if log["event"] == "deleted":
                print("Deleted block {}".format(log["name"]))

signal.signal(signal.SIGINT, signal.SIG_DFL)

context = zmq.Context()

log_client = context.socket(zmq.SUB)
log_client.connect(LOG_SERVER)
log_client.setsockopt_string(zmq.SUBSCRIBE, u"")

# Start async threads
log_thread = _LogThread(log_client)
log_thread.start()

sys.exit(1)
print("done!")

