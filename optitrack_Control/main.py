import client
import sys
from threading import Thread
import time

from NatNetClient import NatNetClient

# current position
pos = [0]*10

# target position
target = []

# First target
target.append([[0.074,0.300,0.048],[0.140,0.300,0.100],[3,3,3],[4,4,4],[5,5,5],[6,6,6],[7,7,7],[8,8,8],[9,9,9],[10,10,10]])
# Second target
target.append([[10,10,10],[2,2,2],[3,3,3],[4,4,4],[5,5,5],[6,6,6],[7,7,7],[8,8,8],[9,9,9],[10,10,10]])


#Input Channel you control 10 Drone
#This sequence is the same Log sequence.
ctrlDrone = ['55','11','0','0','0','0','0','0','0','0']

#==============================================================
#Get optitrack position

def receiveRigidBodyFrame( id, position, rotation ):
    global pos
    pos[id-1] = list(position)

streamingClient = NatNetClient()

#streamingClient.newFrameListener = receiveNewFrame
streamingClient.newFrameListener = None
streamingClient.rigidBodyListener = receiveRigidBodyFrame

streamingClient.run()

#================================================================
#Drone Control

thrust = [0]*10
pitch = [0.0]*10
roll = [0.0]*10
yaw = [0.0]*10

ctrlthread = [0]*10
roll_thread = [0]*10
pitch_thread = [0]*10
yaw_thread = [0]*10
thrust_thread = [0]*10

channelSeq  = [0]*10
connCnt = 0


class ctrlThread(Thread):
    def __init__(self,channel,idx, *args):
        super(ctrlThread, self).__init__(*args)
        global thrust
        global pitch
        global roll
        global yaw
        self.channel = channel
        self.idx = idx
        self.sp = False
    def stop(self):
        self.sp = True
        try:
            self.join()
        except Exception:
            pass

    def run(self):
        while True:
            if (self.sp):
                break
            #print(thrust)
            client.setControl(self.channel, roll[self.idx], pitch[self.idx], yaw[self.idx], thrust[self.idx])
            time.sleep(0.01)


class pidCtrl():

    def __init__(self, desired, sel=0, *args):
        super(pidCtrl, self).__init__(*args)
        self.error = 0.0
        self.prevError = 0.0
        self.integ = 0.0

        #roll, pitch gain
        if sel is 1:
            self.iLimit = 2.0
            self.iLimitLow = -2.0
            self.dt = 10.0
            self.deriv = 0.0
            self.kp = 5.0
            self.ki = 0.05
            self.kd = -10.0
        #thrust gain
        else:
            self.iLimit = 8.0
            self.iLimitLow = -3.0
            self.dt = 10.0
            self.deriv = 0.0
            self.kp = 10.0
            self.ki = 1.0
            self.kd = 600.0
        self.desired = desired
    def pidUpdate(self, measured=0.0):

        self.error = self.desired - measured

        self.integ += self.error * self.dt
        if self.integ > self.iLimit :
            self.integ = self.iLimit
        elif self.integ < self.iLimitLow :
            self.integ = self.iLimitLow

        self.deriv = (self.error - self.prevError) / self.dt

        outP = self.kp * self.error
        outI = self.ki * self.integ
        outD = self.kd * self.deriv

        output = outP + outI + outD
        self.prevError = self.error

        return output


class Roll(Thread):
    def __init__(self, idx, *args):
        super(Roll, self).__init__(*args)
        global target
        global pos
        global roll
        self.target_num = 0
        self.drone_idx = idx
        self.target = target[0][self.drone_idx]
        self.sp = False
        
        self.pid = pidCtrl(self.target[2],1)
    def stop(self):
        self.sp = True
        try:
            self.join()
        except Exception:
            pass

    def change_target(self, num):
        (self.pid).desired = target[num][self.drone_idx][2]

    def change_gain(self, p, i, d):
        (self.pid).kp = p
        (self.pid).ki = i
        (self.pid).kd = d

    def run(self):
        global roll
        while True:
            if (self.sp):
                break
            #roll control
            gap = (self.pid).pidUpdate(float(pos[self.drone_idx][2]))
            roll[0] = -gap
            #print("roll gap:" + str(gap))
            time.sleep(0.01)
            

class Pitch(Thread):
    def __init__(self, idx, *args):
        super(Pitch, self).__init__(*args)
        global target
        global pos
        global pitch
        self.target_num = 0
        self.drone_idx = idx
        self.target = target[0][self.drone_idx]
        self.sp = False

        self.pid = pidCtrl(self.target[0],1)
    def stop(self):
        self.sp = True
        try:
            self.join()
        except Exception:
            pass

    def change_target(self, num):
        (self.pid).desired = target[num][self.drone_idx][0]

    def change_gain(self, p, i, d):
        (self.pid).kp = p
        (self.pid).ki = i
        (self.pid).kd = d

    def run(self):
        global pitch
        while True:
            if (self.sp):
                break
            #pitch control
            gap = (self.pid).pidUpdate(float(pos[self.drone_idx][0]))
            pitch[0] = -gap
            #print("pitch gap:" + str(gap))
            time.sleep(0.01)


class Yaw(Thread):
    def __init__(self, idx, *args):
        super(Yaw, self).__init__(*args)
        global target
        global pos
        global yaw
        self.target_num = 0
        self.drone_idx = idx
        self.target = target[0][self.drone_idx]
        self.sp = False

        self.pid = pidCtrl(self.target[0],1)
    def stop(self):
        self.sp = True
        try:
            self.join()
        except Exception:
            pass

    def change_target(self, num):
        (self.pid).desired = target[num][self.drone_idx][2]

    def change_gain(self, p, i, d):
        (self.pid).kp = p
        (self.pid).ki = i
        (self.pid).kd = d

    def run(self):
        global yaw
        while True:
            if (self.sp):
                break
            #yaw control


class Thrust(Thread):
    def __init__(self, idx, *args):
        super(Thrust, self).__init__(*args)
        global target
        global pos
        global thrust
        self.target_num = 0
        self.drone_idx = idx
        self.target = target[0][self.drone_idx]
        self.sp = False

        self.pid = pidCtrl(self.target[1])
    def stop(self):
        self.sp = True
        try:
            self.join()
        except Exception:
            pass

    def change_target(self, num):
        self.target = target[num][self.drone_idx]

    def change_gain(self, p, i, d):
        (self.pid).kp = p
        (self.pid).ki = i
        (self.pid).kd = d

    def run(self):
        global thrust
        while True:
            if (self.sp):
                break
            #thrust control
            while True:
                gap = (self.pid).pidUpdate(pos[self.drone_idx][1])
                #print(40000+(gap*1000))
                time.sleep(0.01)
                if self.drone_idx is 1:
                    thrust[self.drone_idx] = int(10+(gap*1000))
                elif self.drone_idx is 0:
                    temp = int(40000+(gap*500))
                    if temp >= 60000:
                        temp = 60000
                    print(temp)
                    thrust[self.drone_idx] = temp

#For concurrently connect
class connectThread(Thread):
    def __init__(self, channel, idx, *args):
        super(connectThread, self).__init__(*args)
        global ctrlthread
        global channelSeq
        self.channel = channel
        self.idx = idx
    def run(self):
        #print(self.idx)
        error = client.Connect(self.channel, True, self.idx)
        if error == -1:
            print("connect fail!!!")
            return
        client.setLog(self.channel)
        ctrlthread[self.idx] = ctrlThread(self.channel, self.idx)
        ctrlthread[self.idx].start()
        channelSeq[self.idx] = self.channel

def land():
    global roll
    global pitch
    global yaw
    global thrust
    for i in range(10):
        roll[i] = 0.0
        pitch[i] = 0.0
        yaw[i] = 0.0
        thrust[i] = 0

def findChanidx(channel = ""):
    for i in range(10) :
        if channelSeq[i] == channel :
            return i;
    return -1;


#time.sleep(1)
#print(pos[0])
#print(pos[1])

print("If you don't know command, Enther the 'help'")
while True:
    sel = str(input("<<"))
    if sel == "connect" :
        channel = str(input("Channel input :"))
        error = client.Connect(channel)
        if error == -1:
            print("connect fail!!!")
            continue
        ctrlthread[connCnt] = ctrlThread(channel, connCnt)
        ctrlthread[connCnt].start()
        channelSeq[connCnt] = channel
        connCnt+=1
    elif sel == "disconnect" :
        channel = str(input("Channel input :"))

        idx = findChanidx(channel)
        ctrlthread[idx].stop()
        ctrlthread[idx] = 0

        client.disConnect(channel)
        connCnt-=1
    elif sel == "log" :
        channel = str(input("Channel input :"))
        client.setLog(channel)

    elif sel == "connect all":
        print("1 : At same time 2 : One at a time")
        val = int(input("Input:"))
        if val is 1: 
            cT = [0]*10
            for i in range(2):
                cT[i] = connectThread(ctrlDrone[i], i)
                cT[i].start()

        elif val is 2:
            for i in range(2):
                error = client.Connect(ctrlDrone[i])
                if error == -1:
                    print("connect fail!!!")
                    continue
                ctrlthread[connCnt] = ctrlThread(ctrlDrone[i], connCnt)
                ctrlthread[connCnt].start()
                channelSeq[connCnt] = ctrlDrone[i]
                connCnt+=1


    elif sel == "disconnect all":
        for i in range(2):
            client.disConnect(ctrlDrone[i])
            ctrlthread[i].stop()
        connCnt = 0

    elif sel == "control":
        while(1):
            #Create Drone Operation you want.
            print("1. thrust input mode(hovering)\n2. only thrust mode\n3. auto hovering(height input)\n4. Multi drone control test\n5. Simulation\n7.thrust hovering test\n8.roll,pitch,yaw hovering test")
            try:
                val = int(input("Input : "))
            except Exception as e:
                print("{}".format(e))
                continue
            if val is -1:
                break
            if val is 1 :
                channel = str(input("Channel input :"))
                idx = findChanidx(channel)

                roll_thread[0] = Roll(0)
                pitch_thread[0] = Pitch(0)
                roll_thread[0].start()
                pitch_thread[0].start()
                while(True):
                    print("thrust input:")
                    try:
                        temp = int(input())
                    except Exception as e:
                        print("{}".format(e))
                        continue
                    if temp == -1 :
                        thrust[idx] = 0
                        break
                    thrust[idx] = temp

            elif val is 2 :
                channel = str(input("Channel input :"))
                idx = findChanidx(channel)

                while(True):
                    print("thrust input:")
                    try:
                        temp = int(input())
                    except Exception as e:
                        print("{}".format(e))
                        continue
                    if temp == -1 :
                        thrust[idx] = 0
                        break
                    thrust[idx] = temp

            elif val is 3 :
                channel = str(input("Channel input :"))
                idx = findChanidx(channel)

                roll_thread[0] = Roll(0)
                pitch_thread[0] = Pitch(0)
                thrust_thread[0] = Thrust(0)
                roll_thread[0].start()
                pitch_thread[0].start()
                thrust_thread[0].start()

                temp = str(input("input:"))
                if temp is not None:
                    roll[0] = 0.0
                    pitch[0] = 0.0
                    thrust[0] = 0
                    roll_thread[0].stop()
                    pitch_thread[0].stop()
                    thrust_thread[0].stop()

            elif val is 4 :
                channel = str(input("input first channel:"))
                idx1 = findChanidx(channel)

                channel = str(input("input second channel:"))
                idx2 = findChanidx(channel)

                while(True):
                    print("drone 1 thrust :")
                    try:
                        temp = int(input())
                    except Exception as e:
                        print("{}".format(e))
                        continue
                    if temp == -1 :
                        thrust[idx1] = 0
                        thrust[idx2] = 0
                        break
                    thrust[idx1] = temp

                    print("drone 2 thrust :")
                    try:
                        temp = int(input())
                    except Exception as e:
                        print("{}".format(e))
                        continue
                    if temp == -1 :
                        thrust[idx1] = 0
                        thrust[idx2] = 0
                        break
                    thrust[idx2] = temp

            elif val is 5 :
                for i in range(2):
                    roll_thread[i] = Roll(i)
                    pitch_thread[i] = Pitch(i)
                    yaw_thread[i] = Yaw(i)
                    thrust_thread[i] = Thrust(i)
                    
                    roll_thread[i].start()
                    pitch_thread[i].start()
                    yaw_thread[i].start()
                    thrust_thread[i].start()                    
                while True:
                    num = int(input("Input target number : "))
                    if num is -1:
                        land()
                    for i in range(10):
                        roll_thread[i].change_target(num)
                        pitch_thread[i].change_target(num)
                        yaw_thread[i].change_target(num)
                        thrust_thread[i].change_target(num)

            elif val is 6 :
                thrust[0] = 20000
                while True:
                    thrust[0] = int(input("thrust:"))
                    temp = float(input())
                    pitch[0] = temp

            elif val is 7 :
                channel = str(input("Channel input : "))
                ctrlthread[0] = ctrlThread(channel, 0)
                client.test(channel)
                ctrlthread[0].start()

                channelSeq[0] = channel
                thrust_thread[0] = Thrust(0)
                while True:
                    p = float(input("input p gain:"))
                    i = float(input("input i gain:"))
                    d = float(input("input d gain:"))
                    thrust_thread[0].change_gain(p,i,d)
                    thrust_thread[0].start()
                    a = str(input("If you want to stop, Enter any key"))
                    thrust_thread[0].stop()
                    thrust[0] = 0

            elif val is 8 :
                channel = str(input("Channel input : "))
                ctrlthread[0] = ctrlThread(channel, 0)
                client.test(channel)
                ctrlthread[0].start()

                channelSeq[0] = channel

                pitch_thread[0] = Pitch(0)
                roll_thread[0] = Roll(0)
                yaw_thread[0] = Yaw(0)
                thrust_thread[0] = Thrust(0)
                while True:
                    p = float(input("input p gain:"))
                    i = float(input("input i gain:"))
                    d = float(input("input d gain:"))
                    pitch_thread[0].change_gain(p,i,d)
                    roll_thread[0].change_gain(p,i,d)
                    yaw_thread[0].change_gain(p,i,d)

                    pitch_thread[0].start()
                    roll_thread[0].start()
                    yaw_thread[0].start()
                    thrust_thread[0].start()

                    a = str(input("If you want to stop, Enter any key"))
                    pitch_thread[0].stop()
                    roll_thread[0].stop()
                    yaw_thread[0].stop()
                    thrust_thread[0].stop()
                    pitch[0] = 0
                    roll[0] = 0
                    yaw[0] = 0
                    thrust[0] = 0

    elif sel == "exit" :
        sys.exit(1)
    elif sel == "help" :
        print("command : 1. connect 2. connect all 3. disconnect 4. disconnect all 5. log")
        print("1. connect => Connect selected a drone")
        print("2. connect all => At a time, Connect all drone")
        print("3. disconnect => disconnect selected a drone")
        print("4. disconnect all => At a time, Disconnect all drone")
        print("5. log => Set and Start log to specific port for roll, pitch,yaw,thrust,battery info")
        print("6. control => Operate simulation you create")
        print("If you want to exit, Enter the 'exit'")
    else :
        print("Error : unvalid command")

