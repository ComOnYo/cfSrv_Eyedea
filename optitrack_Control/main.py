import client
import sys
from threading import Thread
import time
import math
import signal
import zmq

from NatNetClient import NatNetClient

# current position
pos = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

# target position(Init position)
target = [[0.19,1.00,0.17],[0.40,1.00,0.30],[3,1.00,3],[4,4,4],[5,5,5],[6,6,6],[7,7,7],[8,8,8],[9,9,9],[10,10,10]]

# velocity
velo_x = [0.0]*10
velo_y = [0.0]*10
velo_z = [0.0]*10
velo_pre_x = [0.0]*10
velo_pre_y = [0.0]*10
velo_pre_z = [0.0]*10

#Input Channel you control 10 Drone
#This sequence is the same Log sequence.
ctrlDrone = ['80','90','50','0','0','0','0','0','0','0']
DroneCnt = 1
#
log_thread = [0]*10
acc_zw = [0]*10

#
hovering_thread = [0]*10
run_chk = 0

#
first_check = [1]*10
optical_flow = 0
optical_pos = [0,0,0]
#==============================================================
#Get optitrack position

def receiveRigidBodyFrame( id, position, rotation ):
    global pos
    global velo_x
    global velo_y
    global velo_z
    global velo_pre_x
    global velo_pre_y
    global velo_pre_z
    global acc_zw
    rateTau = 0.0001

    global hovering_thread
    
    global first_check
    global target

    if first_check[id-1] is 1 :
        target[id-1][0] = position[0]
        target[id-1][2] = position[2]
        print(str(target[0][0]) +", " + str(target[0][2]))
        first_check[id-1]=0

    global optical_flow
    if optical_flow is 1 :
        optical_pos[0] = position[0]
        optical_pos[1] = position[1]
        optical_pos[2] = position[2]

    #print(position)
    #velo[id-1] = (math.sqrt((math.sqrt((position[0]-pos[id-1][0])**2 + (position[2]-pos[id-1][2])**2))**2 + (position[1]-pos[id-1][1])**2)) / (time.time()-velo_time[id-1])
    #pos[id-1] = list(position)
    if run_chk is 1:
        #print(str(target[0][0]) +", " + str(target[0][2]))

        if (pos[id-1][0] is not 0) and (pos[id-1][1] is not 0) and (pos[id-1][2] is not 0) :
            velo_x[id-1] = (position[0]-pos[id-1][0]) / 0.004 #(time.time()-velo_time[id-1])
            velo_y[id-1] = (position[1]-pos[id-1][1]) / 0.004 # (time.time()-velo_time[id-1])
            velo_z[id-1] = (position[2]-pos[id-1][2]) / 0.004 # (time.time()-velo_time[id-1])

        #print("\nvelo_x : {}\n velo_y : {}\n velo_z : {}\nvelo_pre_x : {}\nvelo_pre_y : {}\nvelo_pre_z : {}\nacc_zw : {}\n".format(velo_x[0],velo_y[0],velo_z[0],velo_pre_x[0],velo_pre_y[0],velo_pre_z[0],acc_zw[0]))

        velo_x[id-1] = ((rateTau * velo_pre_x[id-1]) + (0.004 * velo_x[id-1])) / (rateTau + 0.004)
        velo_y[id-1] = ((rateTau * velo_pre_y[id-1]) + (0.004 * velo_y[id-1])) / (rateTau + 0.004)
        velo_z[id-1] = ((rateTau * velo_pre_z[id-1]) + (0.004 * velo_z[id-1])) / (rateTau + 0.004)
 
        velo_pre_x[id-1] = velo_x[id-1]
        velo_pre_y[id-1] = velo_y[id-1]
        velo_pre_z[id-1] = velo_z[id-1]
        #print("velo => " + str(velo_y[id-1]))
        #print("distance => " + str(position[1]))

        pos[id-1] = list(position)
        #print(str(target[0][0]-pos[id-1][0]) + ", " + str(target[0][1]-pos[id-1][1]) + ", " + str(target[0][2]-pos[id-1][2]))

        hovering_thread[id-1].run_()

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

channelSeq  = [0]*10
connCnt = 0

log_thread = [0]*10

class _LogThread(Thread):
    def __init__(self, socket, *args):
        super(_LogThread, self).__init__(*args)
        self._socket = socket
        global acc_zw
        self.zw = 0.0
    def run(self):
        while True:
            log = self._socket.recv_json()
            if log["event"] == "data":
                acc_zw[0] = log["variables"]["acc.WZ"]

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
            #print(str(pitch[self.idx]) + ", " + str(roll[self.idx]) + ", " + str(thrust[self.idx]))
            time.sleep(0.004)


class pidCtrl():

    def __init__(self, num=0, sel=0, *args):
        super(pidCtrl, self).__init__(*args)
        self.error = 0.0
        self.prevError = 0.0
        self.integ = 0.0
        self.x1 = 0.0

        #roll, pitch gain
        if sel is 1:
            self.iLimit = 33.0
            self.iLimitLow = -33.0
            self.dt = 0.004
            self.deriv = 0.0
            self.kp = 1.0
            self.ki = 0.0
            self.kd = 0.0
            self.v_kp = 30.0
            self.v_ki = 30.0 #0.0
            self.v_kd = 1.0
        #thrust gain
        else:
            self.iLimit = 33.0
            self.iLimitLow = -33.0
            self.dt = 0.004
            self.deriv = 0.0
            self.kp = 1.0
            self.ki = 0.0
            self.kd = 0.0
            self.v_kp = 6.0
            self.v_ki = 0.0
            self.v_kd = 0.0
            self.a_kp = 1500   #1500.0
            self.a_ki = 3000  #3000.0
            self.a_kd = 0.0
            self.prevError2 = 0.0
        self.num = num
        self.desired = 0.0
        global target

    def pidUpdate(self, measured=0.0, coordinate = 0):

        self.desired = target[self.num][coordinate]
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

    def pidUpdate2(self, velo = 0.0, desired = 0.0):

        self.desired = desired
        self.error = self.desired - velo

        self.integ += self.error * self.dt
        if self.integ > self.iLimit :
            self.integ = self.iLimit
        elif self.integ < self.iLimitLow :
            self.integ = self.iLimitLow

        self.error = ((0.05*self.prevError)+(self.dt*self.error)) / (0.05+self.dt)
        self.deriv = (self.error - self.prevError) / self.dt

        outP = self.v_kp * self.error
        outI = self.v_ki * self.integ
        outD = self.v_kd * self.deriv

        output = outP + outI + outD
        self.prevError = self.error

        return output

    def pidUpdate3(self, acc_zw = 0.0, desired = 0.0):

        self.desired = desired
        self.error = self.desired - acc_zw

        self.integ += self.error * self.dt
        if self.integ > self.iLimit :
            self.integ = self.iLimit
        elif self.integ < self.iLimitLow :
            self.integ = self.iLimitLow

        self.error = ((0.05*self.prevError)+(self.dt*self.error)) / (0.05+self.dt)
        self.deriv = (self.error - self.prevError) / self.dt

        outP = self.a_kp * self.error
        outI = self.a_ki * self.integ
        outD = self.a_kd * self.deriv

        output = outP + outI + outD
        self.prevError = self.error

        return output

class Hovering(Thread):
    def __init__(self, idx, *args):
        super(Hovering, self).__init__(*args)

        global pos

        global roll
        global pitch
        global yaw
        global thrust

        global acc_zw

        self.drone_idx = idx

        #self.pid_roll = pidCtrl(idx, 1)
        #self.pid_pitch = pidCtrl(idx, 1)
        #self.pid_yaw = pidCtrl(idx, 1)

        #self.pid_thrust = pidCtrl(idx)

        self.pid_roll = pidCtrl(idx,1)
        self.pid_roll2 = pidCtrl(idx, 1)
        self.pid_pitch = pidCtrl(idx, 1)
        self.pid_pitch2 = pidCtrl(idx, 1)
        self.pid_yaw = pidCtrl(idx, 1)
        self.pid_yaw2 = pidCtrl(idx, 1)

        self.pid_thrust = pidCtrl(idx)
        self.pid_thrust2 = pidCtrl(idx)
        self.pid_thrust3 = pidCtrl(idx)

        self.sp = False

    def stop(self):
        self.sp = True
        try:
            self.join()
        except Exception:
            pass

    def change_gain(self, p = 0.0, v_p=0.0, v_i=0.0, v_d=0.0, sel=""):
        if sel is "roll" :
            (self.pid_roll).kp = p
            (self.pid_roll).ki = i
            (self.pid_roll).kd = d
        elif sel is "pitch" :
            (self.pid_pitch).kp = p
            (self.pid_pitch2).v_kp = v_p
            (self.pid_pitch2).v_ki = v_i
            (self.pid_pitch2).v_kd = v_d

            (self.pid_roll).kp = p
            (self.pid_roll2).v_kp = v_p
            (self.pid_roll2).v_ki = v_i
            (self.pid_roll2).v_kd = v_d

        elif sel is "yaw" :
            (self.pid_yaw).kp = p
            (self.pid_yaw).ki = i
            (self.pid_yaw).kd = d
        elif sel is "thrust" :
            (self.pid_thrust2).v_kp = p
            (self.pid_thrust3).a_kp = v_p
            (self.pid_thrust3).a_ki = v_i
            (self.pid_thrust3).a_kd = v_d

    def run_(self):

        #while True:
        #    if (self.sp):
        #        break

        #roll control                                        z   t_z
        gap = (self.pid_roll).pidUpdate(float(pos[self.drone_idx][2]), 2)
        gap = (self.pid_roll2).pidUpdate2(velo_z[self.drone_idx], gap)
        #print(gap)
        roll[self.drone_idx] = -gap
        #print("roll gap:" + str(gap))

        #pitch control                                       x   t_x
        gap = (self.pid_pitch).pidUpdate(float(pos[self.drone_idx][0]), 0)
        gap = (self.pid_pitch2).pidUpdate2(velo_x[self.drone_idx], gap)
        #print(gap)
        pitch[self.drone_idx] = -gap

        #yaw control

        #thrust control
        gap = (self.pid_thrust).pidUpdate(float(pos[self.drone_idx][1]), 1)
        #print(gap)
        #print("")
        gap = (self.pid_thrust2).pidUpdate2(velo_y[self.drone_idx], gap)
        gap = (self.pid_thrust3).pidUpdate3(acc_zw[self.drone_idx], gap)
        #print(gap)
        temp = int(34000 + gap)
        if temp >= 62000:
            temp = 62000
        elif temp <= 0:
            temp = 0
        thrust[self.drone_idx] = temp

        #time.sleep(0.01)


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

        log_thread[connCnt] = _LogThread(client.getLog(channel))
        log_thread[connCnt].start()

        channelSeq[self.idx] = self.channel

def land():
    global target
    for i in range(10):
        target[i][1] = 0.07

def findChanidx(channel = ""):
    for i in range(10) :
        if channelSeq[i] == channel :
            return i;
    return -1;

# Circle      DroneNum, radius
def sequence_1(num = 0, r = 0.0):
    global target
    global pos
    #x^2 + z^2 = r^2
    #init_x = pos[num][0]
    #init_z = pos[num][2]

    init_x = 0.0
    init_z = 0.0

    init_x = target[num][0]
    print(init_z)
    print(r)
    print(target[num][0])
    init_z = math.sqrt(r**2 - target[num][0]**2)
    print(init_z)
    print(r)
    print(target[num][0])

    prove = 0
    minus = 1

    if pos[num][2] > 0:
        prove = 0
    else:
        prove = 1
    
    while True:
        # calc Z    z  =       sqrt(r^2 - x^2)
        if target[num][0] >= r:
            target[num][0] = r
            prove = 0
        elif target[num][0] <= -r:
            target[num][0] = -r
            prove = 1
        
        target[num][2] = math.sqrt(r**2 - target[num][0]**2) * minus

        print("t_x : " + str(target[num][0]) + " t_y : " + str(target[num][2]))
        while True:
            # Check whether really move or not
            #print("x : " + str(target[num][0]) + " z : " + str(target[num][2]))
            #print("pos x : " + str(pos[num][0]) + " pos z : " + str(pos[num][2]))
            '''
            time.sleep(0.004)
            if ((target[num][0]-0.020) < pos[num][0]) and (pos[num][0] < (target[num][0]+0.020)) and ((target[num][2]-0.020) < pos[num][2]) and (pos[num][2] < (target[num][2]+0.020)) :
                # Check up and down based on center.
                if pos[num][2] > 0:
                    target[num][0] -= 0.06
                    minus = 1
                elif pos[num][2] < 0:
                    target[num][0] += 0.06
                    minus = -1
                break
            '''
            time.sleep(0.2)
            if prove is 0:
                target[num][0] -= 0.06
                minus = 1
            elif prove is 1:
                target[num][0] += 0.06
                minus = -1
            break
        # Check one round
        #if (init_x-0.020) < pos[num][0] and pos[num][0] < (init_x+0.020) and (init_z-0.020) < pos[num][2] and pos[num][2] < (init_z+0.020):
        #    break


def init_value():
    global pos
    global velo_x
    global velo_y
    global velo_z
    global velo_pre_x
    global velo_pre_y
    global velo_pre_z
    global acc_zw
    for i in range(10):
        for j in range(3):
            pos[i][j] = 0.0
        velo_x[i] = 0.0
        velo_y[i] = 0.0
        velo_z[i] = 0.0
        velo_pre_x[i] = 0.0
        velo_pre_y[i] = 0.0
        velo_pre_z[i] = 0.0
        acc_zw[i] = 0.0
    
def print_value():
    global velo_x
    global velo_y
    global velo_z
    global velo_pre_x
    global velo_pre_y
    global velo_pre_z
    global acc_zw
    print("velo_x : {}\n velo_y : {}\n velo_z : {}\nvelo_pre_x : {}\nvelo_pre_y : {}\nvelo_pre_z : {}\nacc_zw : {}".format(velo_x[0],velo_y[0],velo_z[0],velo_pre_x[0],velo_pre_y[0],velo_pre_z[0],acc_zw[0]))

def SetTarget(num = 0, x = 0.0, y = 0.0, z = 0.0):
    global target
    target[num][0] = x
    target[num][1] = y
    target[num][2] = z

def location_debug():
    global target
    global pos
    print("target : " + str(target[0][0]) + " " + str(target[0][1]) + " " + str(target[0][2]))
    print("pos : " + str(pos[0][0]) + " " + str(pos[0][1]) + " " + str(pos[0][2]))
    global pitch
    global roll
    print("pitch : " + str(pitch[0]) + " " + "roll : " + str(roll[0]))

#time.sleep(1)
#print(pos[0])
#print(pos[1])

print("If you don't know command, Enther the 'help'")
while True:
    global DroneCnt

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

        client.setLog(channel)
        log_thread[connCnt] = _LogThread(client.getLog(channel))
        log_thread[connCnt].start()

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
            for i in range(DroneCnt):
                cT[i] = connectThread(ctrlDrone[i], i)
                cT[i].start()

        elif val is 2:
            for i in range(DroneCnt):
                error = client.Connect(ctrlDrone[i])
                if error == -1:
                    print("connect fail!!!")
                    continue
                ctrlthread[connCnt] = ctrlThread(ctrlDrone[i], connCnt)
                ctrlthread[connCnt].start()

                client.setLog(ctrlDrone[i])
                log_thread[connCnt] = _LogThread(client.getLog(ctrlDrone[i]))
                log_thread[connCnt].start()

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
            print("1. thrust input mode(hovering)\n2. only thrust mode\n3. auto hovering(height input)\n4. Multi drone control test\n5. Simulation\n7.hovering test(Play Only one drone)\n8.roll,pitch,yaw hovering test(Play Only one drone)")
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

                temp = str(input("To stop, any input:"))
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
                global run_chk
                global target
                global thrust
                for i in range(DroneCnt):
                    hovering_thread[i] = Hovering(i)

                while True:
                    str(input("Start any key"))
                    #thrust[0] = 50000
                    #time.sleep(0.5)

                    run_chk = 1
                    target[0][1] = 1.0
                    while True:
                        print("1. Move 2. land 3. Fly off")
                        i = str(input("Input : "))
                        if i is '1':
                            while True:
                                print("1. left 2. right 3. front 4. back 5.stop 6.circle 7.location_debug 8.setTarget")
                                j = str(input("input : "))
                                if j is '1':
                                    print("left")
                                    SetTarget(0, target[0][0]-0.30, target[0][1], target[0][2])
                                    SetTarget(1, target[1][0]-0.30, target[1][1], target[1][2])
                                    SetTarget(2, target[2][0]-0.30, target[2][1], target[2][2])
                                elif j is '2':
                                    print("right")
                                    SetTarget(0, target[0][0]+0.30, target[0][1], target[0][2])
                                    SetTarget(1, target[1][0]+0.30, target[1][1], target[1][2])
                                    SetTarget(2, target[2][0]+0.30, target[2][1], target[2][2])
                                elif j is '3':
                                    print("front")
                                    SetTarget(0, target[0][0], target[0][1], target[0][2]+0.30)
                                    SetTarget(1, target[1][0], target[1][1], target[1][2]+0.30)
                                    SetTarget(2, target[2][0], target[2][1], target[2][2]+0.30)
                                elif j is '4':
                                    print("back")
                                    SetTarget(0, target[0][0], target[0][1], target[0][2]-0.30)
                                    SetTarget(1, target[1][0], target[1][1], target[1][2]-0.30)
                                    SetTarget(2, target[2][0], target[2][1], target[2][2]-0.30)
                                elif j is '5':
                                    print("exit")
                                    break
                                elif j is '6':
                                    sequence_1(0,0.60)
                                elif j is '7':
                                    location_debug()
                                elif j is '8':
                                    x = float(input("X : "))
                                    y = float(input("Y : "))
                                    z = float(input("Z : "))
                                    SetTarget(0,x,y,z)
                        elif i is '2':
                            land()
                        elif i is '3':
                            run_chk = 0
                            #init_value()
                            for i in range(DroneCnt):
                                pitch[i] = 0
                                roll[i] = 0
                                yaw[i] = 0
                                thrust[i] = 0
                            break

            elif val is 6 :
                '''
                global thrust
                global pitch
                thrust[0] = 20000
                while True:
                    #thrust[0] = int(input("thrust:"))
                    temp = float(input())
                    pitch[0] = temp
                '''
                channel = str(input("Channel input : "))
                ctrlthread[0] = ctrlThread(channel, 0)
                client.test(channel)
                ctrlthread[0].start()

                channelSeq[0] = channel
                hovering_thread[0] = Hovering(0)

                #Test
                #while True:
                #    thrust[0] = int(input("Input thrust : "))
                global run_chk
                while True:
                    #p = float(input("input p gain:"))
                    #v_p = float(input("input vp gain:"))
                    #v_i = float(input("input vi gain:"))
                    #v_d = float(input("input vd gain:"))

                    #hovering_thread[0] = Hovering(0)
                    #hovering_thread[0].change_gain(p,v_p,v_i,v_d,"pitch")
                    #hovering_thread[0].change_gain(v_p,v_i,v_d,"thrust")

                    str(input("Start any key"))
                    run_chk = 1
                    #hovering_thread[0].start()
                    while True:
                        a = str(input("If you want to stop, Enter any key"))
                        if a is 'w':
                            pitch[0] += 5.0
                        elif a is 's':
                            pitch[0] -= 5.0
                        elif a is 'q':
                            run_chk = 0
                            #hovering_thread[0].stop()
                            pitch[0] = 0
                            roll[0] = 0
                            yaw[0] = 0
                            thrust[0] = 0
                            break
                            #hovering_thread[0] = None

            elif val is 7 :
                '''
                channel = str(input("Channel input : "))
                ctrlthread[0] = ctrlThread(channel, 0)
                client.test(channel)
                ctrlthread[0].start()

                channelSeq[0] = channel
                '''
                hovering_thread[0] = Hovering(0)
                
                #Test
                #while True:
                #    thrust[0] = int(input("Input thrust : "))
                global run_chk
                while True:
                    p = float(input("input p gain:"))
                    v_p = float(input("input vp gain:"))
                    v_i = float(input("input vi gain:"))
                    v_d = float(input("input vd gain:"))

                    hovering_thread[0] = Hovering(0)
                    hovering_thread[0].change_gain(p,v_p,v_i,v_d,"pitch")
                    #hovering_thread[0].change_gain(p,v_p,v_i,v_d,"thrust")

                    target[0][1] = 1.0
                    run_chk = 1
                    #hovering_thread[0].start()
                    while True:
                        print("1. left 2. right 3.circle 4.location_debug 5.stop")
                        j = str(input("input : "))
                        if j is '1':
                            print("left")
                            SetTarget(0, target[0][0]-0.30, target[0][1], target[0][2])
                            SetTarget(1, target[1][0]-0.30, target[1][1], target[1][2])
                            SetTarget(2, target[2][0]-0.30, target[2][1], target[2][2])
                        elif j is '2':
                            print("right")
                            SetTarget(0, target[0][0]+0.30, target[0][1], target[0][2])
                            SetTarget(1, target[1][0]+0.30, target[1][1], target[1][2])
                            SetTarget(2, target[2][0]+0.30, target[2][1], target[2][2])
                        elif j is '3':
                            sequence_1(0,0.30)
                        elif j is '4':
                            location_debug()
                        elif j is '5':
                            land()
                            str(input("enter"))
                            run_chk = 0
                            #init_value()
                            #hovering_thread[0].stop()
                            pitch[0] = 0
                            roll[0] = 0
                            yaw[0] = 0
                            thrust[0] = 0
                            #hovering_thread[0] = None
                            break

            elif val is 8 :
                global run_chk
                global target
                global pos
                global optical_flow
                global optical_pos

                hovering_thread[0] = Hovering(0)
                while True:
                    optical_flow = 1
                    a = str(input("If you want to start, Enter any key"))
                    hovering_thread[0] = Hovering(0)
                    run_chk = 1
                    #optical_flow = 1
                    target[0][0] = optical_pos[0]
                    target[0][1] = optical_pos[1]
                    target[0][2] = optical_pos[2]

                    print(str(target[0][0]) + " " + str(target[0][1]) + " " + str(target[0][2]))
                    
                    a = str(input("If you want to stop, Enter any key"))
                    run_chk = 0

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

