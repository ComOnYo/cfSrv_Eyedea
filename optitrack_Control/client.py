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

def seqEmptyChk():
    for i in range(DRONE_CNT):
        if channelSeq[i] is 0:
            return i


def Connect(channel="", All=False, _idx = 0):

    if All == True :
        cmdSocket[_idx] = context.socket(zmq.REQ)
        cmdSocket[_idx].connect(command.ServerAddr("cmd", str(_idx)))
        cmdSocket[_idx].send_json(command.connDrone(channel))
        resp = cmdSocket[_idx].recv_json()
        if resp["status"] != 0:
            print("fail! {}".format(resp["msg"]))
            return -1
        print(channel + " Connect done!")

        ctrlSocket[_idx] = context.socket(zmq.PUSH)
        ctrlSocket[_idx].connect(command.ServerAddr("ctrl", str(_idx)))

        channelSeq[_idx] = channel
        return

    global connCnt

    if connCnt >= DRONE_CNT :
        print("Full connection!!!")
        return
    elif findChanidx(channel) != -1 :
        print("Channel : " + channel + " exist Link!!!")
        return

    idx = seqEmptyChk()

    cmdSocket[idx] = context.socket(zmq.REQ)
    cmdSocket[idx].connect(command.ServerAddr("cmd", str(idx)))
    cmdSocket[idx].send_json(command.connDrone(channel))
    resp = cmdSocket[idx].recv_json()
    if resp["status"] != 0:
        print("fail! {}".format(resp["msg"]))
        return -1
    print(channel + " Connect done!")

    ctrlSocket[idx] = context.socket(zmq.PUSH)
    ctrlSocket[idx].connect(command.ServerAddr("ctrl", str(idx)))

    channelSeq[idx] = channel
    connCnt+=1

def disConnect(channel=""):
    global connCnt

    idx = findChanidx(channel)
    if idx == -1 :
        print("Channel : " + channel + " not exist Link!!!")
        return

    cmdSocket[idx].send_json(command.disconnDrone(channel))
    resp = cmdSocket[idx].recv_json()
    if resp["status"] != 0:
        print("fail! {}".format(resp["msg"]))
        sys.exit(1)
    print(channel + " disConnect done!")

    cmdSocket[idx].close()
    cmdSocket[idx] = None
    ctrlSocket[idx].close()
    ctrlSocket[idx] =None
    channelSeq[idx] = 0

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
        print(channel + " channel is not exist")
        return
    json = command.controlJson(roll, pitch, yaw, thrust)
    #print(json)
    ctrlSocket[idx].send_json(json)

def test(channel=""):
    ctrlSocket[0] = context.socket(zmq.PUSH)
    ctrlSocket[0].connect(command.ServerAddr("ctrl", '0'))
    channelSeq[0] = channel

