from __future__ import print_function
from threading import Thread
import signal
import time
import sys
import zmq

import command

signal.signal(signal.SIGINT, signal.SIG_DFL)

context = zmq.Context()

DRONE_CNT = 10

channelSeq = [0] * DRONE_CNT
cmdSocket = [0] * DRONE_CNT
ctrlSocket = [0] * DRONE_CNT

connCnt = 0

def findChanidx(channel=""):
    for i in range(DRONE_CNT):
        if channelSeq[i] == channel:
            return i
    return -1

def Connect(channel=""):
    global connCnt
    global cmdSocket

    cmdSocket[connCnt] = context.socket(zmq.REQ)
    cmdSocket[connCnt].connect(command.ServerAddr("cmd", str(connCnt)))
    cmdSocket[connCnt].send_json(command.connDrone(channel))
    resp = cmdSocket[connCnt].recv_json()
    if resp["status"] != 0:
        print("fail! {}".format(resp["msg"]))
        sys.exit(1)
    print(channel + " Connect done!")

    ctrlSocket[connCnt] = context.socket(zmq.PUSH)
    ctrlSocket[connCnt].connect(command.ServerAddr("ctrl", str(connCnt)))

    channelSeq[connCnt] = channel
    connCnt+=1

def disConnect(channel=""):
    client = context.socket(zmq.REQ)
    client.connect(command.ServerAddr("cmd", '0'))
    client.send_json(command.connDrone(channel))
    resp = client.recv_json()
    if resp["status"] != 0:
        print("fail! {}".format(resp["msg"]))
        sys.exit(1)
    print(channel + " disConnect done!")
    connCnt-=1
#=================================================
def setLog(channel="") :

    idx = findChanidx(channel)
    if idx == -1 :
        print("channel is not exist")
        return

    cmdSocket[idx].send_json(command.log_create)
    resp = cmdSocket[idx].recv_json()
    if resp["status"] == 0:
        print("done!")
    else:
        print("fail! {}".format(resp["msg"]))

    cmdSocket[idx].send_json(command.log_start)
    resp = cmdSocket[idx].recv_json()
    if resp["status"] == 0:
        print("done!")
    else:
        print("fail!")

def setControl(channel="", roll=0.0, pitch=0.0, yaw=0.0, thrust=0):
    idx = findChanidx(channel)
    if idx == -1 :
        print(channel + " channel is nod exit")
        return
    json = command.controlJson(roll, pitch, yaw, thrust)
    #print(json)
    ctrlSocket[idx].send_json(json)

