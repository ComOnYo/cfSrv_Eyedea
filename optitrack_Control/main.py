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
target = [[0.00,0.70,0.00],[0.00,0.7,0.00],[0.00,0.7,0.00],[0.0,0.70,0.0],[0.0,0.70,0.0],[0.0,0.70,0.0],[0.0,0.7,0.0],[0.0,0.7,0.0],[0.0,0.7,0.0],[0.0,0.7,0.0]]

# velocity
velo_x = [0.0]*10
velo_y = [0.0]*10
velo_z = [0.0]*10
velo_pre_x = [0.0]*10
velo_pre_y = [0.0]*10
velo_pre_z = [0.0]*10

#Input Channel you control 10 Drone
#This sequence is the same Log sequence.
ctrlDrone = ['54','30','99','125','125','99','113','125','0']
DroneCnt = 2
#
log_socket = [0]*10
#log_thread = [0]*10
log_thread = 0.0
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

yaw_target = [2.0]*10
yaw_value = [0.0]*10
yaw_prev = [0.0]*10
gyro_z = [0.0]*10

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

    global yaw_value
    global yaw_prev
     
    if first_check[id-1] is 1 :
        target[id-1][0] = position[0]
        target[id-1][2] = position[2]
        print(str(target[id-1][0]) +", " + str(target[id-1][2]))
        first_check[id-1]=0
    
    global optical_flow
    if optical_flow is 1 :
        optical_pos[0] = position[0]
        optical_pos[1] = position[1]
        optical_pos[2] = position[2]

    #euler = tf.transformations.euler_from_quaternion(rotation)
    #qw = rotation[0]
    qx = rotation[0]
    qy = rotation[1]
    qz = rotation[2]
    qw = rotation[3]

    #temp = math.atan2(2.0*(qy*qz+qw*qx),qw*qw-qx*qx-qy*qy+qz*qz)
    temp = math.atan2(2.0*(qx*qz+qw*qy),qw*qw-qx*qx-qy*qy+qz*qz)
    temp2 = temp * 180.0 /  3.141592

    temp2 *= -1   

    #print(temp2)
 
    yaw_value[id-1] = temp2

    #temp2 = 0.0
#    if temp2 < 0:
#        yaw_value[id-1] = temp2+360
#    else:
#        yaw_value[id-1] = temp2

#    if 0 <= yaw_value[id-1] and yaw_value[id-1] < 180:
#        yaw_value[id-1] += 180
#    else :
#        yaw_value[id-1] -= 180

    #print(id, end=" ")
    #print(yaw_value[id-1])

    #print(position)
    #pos[id-1] = list(position)
    #print(run_chk)
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

        #print("laskdjf")
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

ctrlthread = [0]
#ctrlthread = [0]*10

channelSeq  = [0]*10
connCnt = 0


class _LogThread(Thread):
    def __init__(self, socket, num, *args):
        super(_LogThread, self).__init__(*args)
        self._socket = [0]*10
        self.num = num

        for i in range(num):
            self._socket[i] = socket[i]
        global acc_zw
        global gyro_z
        #self.zw = 0.0
    def run(self):
        while True:
            for i in range(self.num):
                log = self._socket[i].recv_json()
                if log["event"] == "data":
                    acc_zw[i] = log["variables"]["acc.WZ"]
                    gyro_z[i] = log["variables"]["gyro.z"]
                    #acc_zw[self.num] = log["variables"]["acc.WZ"]
                    #gyro_z[self.num] = log["variables"]["gyro.z"]
                    #print("asdf")

class ctrlThread(Thread):
    def __init__(self, *args):
        super(ctrlThread, self).__init__(*args)
        global thrust
        global pitch
        global roll
        global yaw

        global ctrlDrone
        global DroneCnt
        #self.channel = channel
        #self.idx = idx
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
            #a = time.time()
            for i in range(DroneCnt):
                client.setControl(ctrlDrone[i], roll[i], pitch[i], yaw[i], thrust[i])
                #print(i, end=": ")
                #print(str(pitch[i]) + ", " + str(roll[i]) + ", " + str(thrust[i]))
            time.sleep(0.004)
            #print(time.time() - a)


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
            self.kp = 5	#small 1.0 / big 1
            self.ki = 0.0
            self.kd = 0.0
            self.v_kp = 25.0  #40.0 #20 /10.0 small 30.0 / big 50.0
            self.v_ki = 10.0 #20.0 #30 /21.0 small 30.0 / big 50.0
            self.v_kd = 3.0 #1.0 #6 /4.0 small 1.0 / big 5.0
        #yaw gain
        elif sel is 2:
            self.iLimit = 33.0
            self.iLimitLow = -33.0
            self.dt = 0.004
            self.deriv = 0.0

            self.y_kp = 5
            self.y_ki = 0
            self.y_kd = 0
            self.yv_kp = 0.5
            self.yv_ki = 0.5
            self.yv_kd = 0
        #thrust gain
        else:
            self.iLimit = 33.0
            self.iLimitLow = -33.0
            self.dt = 0.004
            self.deriv = 0.0
            self.kp = 1.0
            self.ki = 0.0
            self.kd = 0.0
            self.v_kp = 6.0 # small 6.0 / big 3.0
            self.v_ki = 0.0
            self.v_kd = 0.0
            self.a_kp = 1500   #small 1500.0 / big 4200
            self.a_ki = 3000  #small 3000.0 / big 5000
            self.a_kd = 100.0 #small 100.0 / big 50.0
            self.prevError2 = 0.0
        self.num = num
        self.desired = 0.0
        global target

        self.sel = sel
    def pidUpdate(self, measured=0.0, coordinate = 0):

        self.desired = target[self.num][coordinate]
        self.error = self.desired - measured
        #if self.sel is 0:
        #    print("desired :" + str(self.desired) + " measured:"+str(measured))

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

    def pidUpdate_yaw(self, measured=0.0, tar = 0.0, sel = 0):

        self.desired = tar
        self.error = self.desired - measured
        #if self.sel is 0:
        #    print("desired :" + str(self.desired) + " measured:"+str(measured))

        self.integ += self.error * self.dt
        if self.integ > self.iLimit :
            self.integ = self.iLimit
        elif self.integ < self.iLimitLow :
            self.integ = self.iLimitLow

        self.deriv = (self.error - self.prevError) / self.dt

        if sel is 0:
            outP = self.y_kp * self.error
            outI = self.y_ki * self.integ
            outD = self.y_kd * self.deriv
        else:
            outP = self.yv_kp * self.error
            outI = self.yv_ki * self.integ
            outD = self.yv_kd * self.deriv

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

        global yaw_value
        global yaw_target
        self.drone_idx = idx

        self.pid_roll = pidCtrl(idx,1)
        self.pid_roll2 = pidCtrl(idx, 1)
        self.pid_pitch = pidCtrl(idx, 1)
        self.pid_pitch2 = pidCtrl(idx, 1)
        self.pid_yaw = pidCtrl(idx, 2)
        self.pid_yaw2 = pidCtrl(idx, 2)

        self.pid_thrust = pidCtrl(idx,3)
        self.pid_thrust2 = pidCtrl(idx,3)
        self.pid_thrust3 = pidCtrl(idx,3)

        self.sp = False

        print(self.drone_idx)
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

        a = time.time()
        #roll control                                        z   t_z
        #print("z : " + str(roll_value), end=" ")
        gap = (self.pid_roll).pidUpdate(float(pos[self.drone_idx][2]), 2)
        #print(" roll error : " + str((self.pid_roll).error))
        gap = (self.pid_roll2).pidUpdate2(velo_z[self.drone_idx], gap)
        
        #print(gap)
        if gap >= 40.0:
            gap = 40.0
        elif gap <= -40.0:
            gap = -40.0
        roll_gap = gap

        #roll[self.drone_idx] = -gap
        #print(velo_z[self.drone_idx], end=" ")
        #print("roll gap:" + str(-gap))

        #pitch control                                       x   t_x
        #print("x : " + str(pitch_value), end=" ")
        gap = (self.pid_pitch).pidUpdate(float(pos[self.drone_idx][0]), 0)
        #print(" pitch error : " + str((self.pid_pitch).error))
        gap = (self.pid_pitch2).pidUpdate2(velo_x[self.drone_idx], gap)
        #print(" pitch error : " + str((self.pid_pitch2).error) + " " + str((self.pid_pitch2).integ) + " " + str((self.pid_pitch2).deriv))
        if gap >= 40.0:
            gap = 40.0
        elif gap <= -40.0:
            gap = -40.0
        pitch_gap = gap
        #print(velo_x[self.drone_idx], end=" ")
        #print("pitch " + str(-gap))
        #pitch[self.drone_idx] = -gap
        
        roll[self.drone_idx] = -(roll_gap * math.cos(yaw_value[self.drone_idx]*3.141592/180) - pitch_gap * math.sin(yaw_value[self.drone_idx]*3.141592/180))
        pitch[self.drone_idx] = -(pitch_gap * math.cos(yaw_value[self.drone_idx]*3.141592/180) + roll_gap * math.sin(yaw_value[self.drone_idx]*3.141592/180))

        #print("roll :" + str(roll[self.drone_idx]), end = " ")
        #print("pitch :" + str(pitch[self.drone_idx]))

        #yaw control
        #print("input : " + str(yaw_value[self.drone_idx]) + " target : " + str(yaw_target[self.drone_idx]), end=" ")
        gap = (self.pid_yaw).pidUpdate_yaw(float(yaw_value[self.drone_idx]), yaw_target[self.drone_idx], 0)

        #print(" gyro : " + str(gyro_z[self.drone_idx]) + " gap : " + str(gap), end="")
        gap = (self.pid_yaw2).pidUpdate_yaw(gyro_z[self.drone_idx], gap, 1)
        if gap >= 180.0:
            gap = 180.0
        elif gap <= -180.0:
            gap = -180.0
        #print("yaw " + str(gap))

        yaw[self.drone_idx] = gap

        #thrust control
        gap = (self.pid_thrust).pidUpdate(float(pos[self.drone_idx][1]), 1)
        gap = (self.pid_thrust2).pidUpdate2(velo_y[self.drone_idx], gap)
        gap = (self.pid_thrust3).pidUpdate3(acc_zw[self.drone_idx], gap)
        #print(acc_zw[self.drone_idx])
        #gap = (self.pid_thrust3).pidUpdate3(0.0, gap)
        #print(gap)
        temp = int(37000 + gap)
        if temp >= 62000:
            temp = 62000
        elif temp <= 0:
            temp = 0
        #print(temp)
        thrust[self.drone_idx] = temp
        
        #print(time.time() - a)


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
        error = client.Connect(self.channel, False, self.idx)
        if error == -1:
            print("connect fail!!!")
            return

        '''
        client.setLog(self.channel)
        ctrlthread[self.idx] = ctrlThread(self.channel, self.idx)
        ctrlthread[self.idx].start()

        log_thread[connCnt] = _LogThread(client.getLog(channel))
        log_thread[connCnt].start()
        '''
        channelSeq[self.idx] = self.channel

def land():
    global target
    for i in range(10):
        target[i][1] = 0.01

def findChanidx(channel = ""):
    for i in range(10) :
        if channelSeq[i] == channel :
            return i;
    return -1;

# Circle      DroneNum, radius
class sequence_1(Thread):
    def __init__(self, num, r, psi, current=0, *args):
        super(sequence_1, self).__init__(*args)
        global pos
        global target

        self.num = num
        self.r = r
        self.cnt = 0.0
        self.sp = False

        self.rotation_cnt = 10 * 360.0
        self.psi = psi

        self.x = pos[self.num][0]
        self.z = pos[self.num][2]

        self.current = current

        if current == 1:
            target[self.num][0] = self.x + self.r*(math.cos(self.psi*3.141592/180))
            target[self.num][2] = self.z + self.r*(math.sin(self.psi*3.141592/180))
        else:
            target[self.num][0] = self.r*(math.cos(self.psi*3.141592/180))
            target[self.num][2] = self.r*(math.sin(self.psi*3.141592/180))
    def stop(self):
        self.sp = True
        try:
            self.join()
        except Exception:
            pass

    def run(self):
        global target
        global pos
        #x^2 + z^2 = r^2
        #init_x = pos[num][0]
        #init_z = pos[num][2]

        #self.x = pos[self.num][0]
        #self.z = pos[self.num][2]

        chk = 0
        while True:
            if (self.sp):
                break

            if self.current == 1:
                target[self.num][0] = self.x + self.r*(math.cos(self.psi*3.141592/180))
                target[self.num][2] = self.z + self.r*(math.sin(self.psi*3.141592/180))
            else:
                target[self.num][0] = self.r*(math.cos(self.psi*3.141592/180))
                target[self.num][2] = self.r*(math.sin(self.psi*3.141592/180))

            self.psi += 0.5
            self.cnt += 0.5
            if self.cnt >= self.rotation_cnt:
                break
            #elif self.cnt % 30.0 == 0.0:
            #    if chk is 0:
            #        target[self.num][1] += 0.30
            #        chk = 1
            #    else:
            #        target[self.num][1] -= 0.30
            #        chk = 0

            if self.psi >= 360.0:
                self.psi = 0.0
            time.sleep(0.01)

class Triangle(Thread):
    
    def __init__(self, num, r, point, *args):
        super(Triangle, self).__init__(*args)
        self.sp = False
        self.num = num
        self.r = r
        self.chk = 0

        global target
        global pos

        if point == 0:
            target[self.num][0] = r/2
            target[self.num][2] = self.r * math.cos(30*3.141592/180)/2
            self.chk = 1
            self.temp = target[self.num][2]
            self.temp2 = -target[self.num][2]
        elif point == 1:
            target[self.num][0] = 0.0
            target[self.num][2] = -self.r * math.cos(30*3.141592/180)/2
            self.chk = 2
            self.temp = -target[self.num][2]
            self.temp2 = target[self.num][2]
        elif point == 2:
            target[self.num][0] = -r/2
            target[self.num][2] = self.r * math.cos(30*3.141592/180)/2
            self.chk = 0
            self.temp = target[self.num][2]
            self.temp2 = -target[self.num][2]
        #self.temp = target[self.num][2]
        #self.temp2 = -target[self.num][2]
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

            if self.chk is 0:
                target[self.num][0] += 0.02
                target[self.num][0] = round(target[self.num][0],3)
                print("x : " + str(target[self.num][0]) + " z : " + str(target[self.num][2]))
                if target[self.num][0] == (self.r)/2 :
                    self.chk = 1

            elif self.chk is 1:
                target[self.num][0] -= 0.01
                target[self.num][2] = self.temp - math.tan(60*3.141592/180)*(self.r/2-target[self.num][0])
                target[self.num][0] = round(target[self.num][0],3)
                print(self.r)
                print(target[self.num][0])
                print("x : " + str(target[self.num][0]) + "z : " + str(target[self.num][2]))
                if target[self.num][0] == 0:
                    self.chk = 2

            elif self.chk is 2:
                target[self.num][0] -= 0.01
                target[self.num][2] = self.temp2 - math.tan(60*3.141592/180)*(target[self.num][0])
                target[self.num][0] = round(target[self.num][0],3)
                print("x : " + str(target[self.num][0]) + " z : " + str(target[self.num][2]))
                if target[self.num][0] == -((self.r)/2):
                    self.chk = 0
            time.sleep(0.04)
            #target[self.num][0] = 
            #target[self.num][1] = 

class Rectangle(Thread):
    def __init__(self, num, width, height, point, *args):
        super(Rectangle, self).__init__(*args)
        self.sp = False
        self.num = num
        self.width = width
        self.height = height
        self.chk = 0

        global target
        global pos

        if point == 0:
            target[self.num][0] = width/2.0
            target[self.num][2] = height/2.0
            self.chk = 1
        elif point == 1:
            target[self.num][0] = width/2.0
            target[self.num][2] = -height/2.0
            self.chk = 2
        elif point == 2:
            target[self.num][0] = -width/2.0
            target[self.num][2] = -height/2.0
            self.chk = 3
        elif point == 3:
            target[self.num][0] = -width/2.0
            target[self.num][2] = height/2.0
            self.chk = 0

        #target[self.num][0] = 0.0
        #target[self.num][2] = height/2.0
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

            if self.chk is 0:
                target[self.num][0] += 0.01
                target[self.num][0] = round(target[self.num][0],3)
                print("x : " + str(target[self.num][0]))
                if target[self.num][0] == (self.width)/2 :
                    self.chk = 1

            elif self.chk is 1:
                target[self.num][2] -= 0.01
                target[self.num][2] = round(target[self.num][2],3)
                print("z : " + str(target[self.num][2]))
                if target[self.num][2] == -((self.height)/2):
                    self.chk = 2

            elif self.chk is 2:
                target[self.num][0] -= 0.01
                target[self.num][0] = round(target[self.num][0],3)
                print("x : " + str(target[self.num][0]))
                if target[self.num][0] == -((self.width)/2):
                    self.chk = 3

            elif self.chk is 3:
                target[self.num][2] += 0.01
                target[self.num][2] = round(target[self.num][2],3)
                print("z : " + str(target[self.num][2]))
                if target[self.num][2] == (self.height)/2:
                    self.chk = 0

            time.sleep(0.03)


def P_1():
    global target

    print("start p_1")
    SetTarget(0, 0.40, target[0][1], 0)
    time.sleep(2)

    SetTarget(1, 0.15, target[1][1], 0)
    time.sleep(2)

    SetTarget(2, -0.15, target[2][1], 0)
    time.sleep(2)

    SetTarget(3, -0.40, target[3][1], 0)
    time.sleep(2)
def P_2():
    global target

    print("start p_2")
    SetTarget(0, 0.40, target[0][1], 0.40)
    SetTarget(1, 0.15, target[1][1], 0.40)
    SetTarget(2, -0.15, target[2][1], 0.40)
    SetTarget(3, -0.40, target[3][1], 0.40)

    time.sleep(2)
    
    SetTarget(0, 0.40, target[0][1], 0.0)
    SetTarget(1, 0.15, target[1][1], 0.0)
    SetTarget(2, -0.15, target[2][1], 0.0)
    SetTarget(3, -0.40, target[3][1], 0.0)

    time.sleep(2)

    SetTarget(0, 0.40, target[0][1], -0.40)
    SetTarget(1, 0.15, target[1][1], -0.40)
    SetTarget(2, -0.15, target[2][1], -0.40)
    SetTarget(3, -0.40, target[3][1], -0.40)

    time.sleep(2)

    SetTarget(0, 0.40, target[0][1], 0.0)
    SetTarget(1, 0.15, target[1][1], 0.0)
    SetTarget(2, -0.15, target[2][1], 0.0)
    SetTarget(3, -0.40, target[3][1], 0.0)

def init_value():
    #global pos
    global velo_x
    global velo_y
    global velo_z
    global velo_pre_x
    global velo_pre_y
    global velo_pre_z
    global acc_zw
    for i in range(10):
        #for j in range(3):
        #    pos[i][j] = 0.0
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
        log_thread[connCnt] = _LogThread(client.getLog(channel), connCnt)
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

            time.sleep(3)
            ctrlthread = ctrlThread()
            ctrlthread.start()


            for i in range(DroneCnt):
                client.setLog(ctrlDrone[i])
                log_socket[i] = client.getLog(ctrlDrone[i])

            log_thread = _LogThread(log_socket, DroneCnt)
            log_thread.start()

        elif val is 2:
            for i in range(DroneCnt):
                error = client.Connect(ctrlDrone[i])
                if error == -1:
                    print("connect fail!!!")
                    continue
                channelSeq[connCnt] = ctrlDrone[i]
                connCnt+=1

                #if i == 0:
                #    ctrlthread[connCnt] = ctrlThread(ctrlDrone[i], connCnt)
                #    ctrlthread[connCnt].start()
            ctrlthread = ctrlThread()
            ctrlthread.start()


            for i in range(DroneCnt):
                client.setLog(ctrlDrone[i])
                log_socket[i] = client.getLog(ctrlDrone[i])

            log_thread = _LogThread(log_socket, DroneCnt)
            log_thread.start()

                #client.setLog(ctrlDrone[i])
                #log_thread[connCnt] = _LogThread(client.getLog(ctrlDrone[i]), i)
                #log_thread[connCnt].start()

                #channelSeq[connCnt] = ctrlDrone[i]
                #connCnt+=1


    elif sel == "disconnect all":
        for i in range(2):
            client.disConnect(ctrlDrone[i])
            ctrlthread[i].stop()
        connCnt = 0

    elif sel == "control":
        while(1):
            #Create Drone Operation you want.
            print("1. only thrust mode\n2. Simulation\n3.hovering test(Play Only one drone)")
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
                global run_chk
                global target
                global thrust
                for i in range(DroneCnt):
                    hovering_thread[i] = Hovering(i)

                while True:
                    str(input("Start any key"))

                    run_chk = 1
                    #target[0][1] = 0.70
                    #target[1][1] = 0.70
                    print(str(target[0][0]) + " " + str(target[0][1]) + " " + str(target[0][2]))
                    while True:
                        print("1. Move 2. land 3. Fly off")
                        i = str(input("Input : "))
                        if i is '1':
                            while True:
                                print("1. left 2. right 3. front 4. back 5.stop 6.circle 7.location_debug 8.setTarget 9. yaw input 10.Rectangle")
                                j = str(input("input : "))
                                print(j)
                                if j is '1':
                                    print("left")
                                    SetTarget(0, target[0][0]-0.50, target[0][1], target[0][2])
                                    SetTarget(1, target[1][0]-0.50, target[1][1], target[1][2])
                                    SetTarget(2, target[2][0]-0.50, target[2][1], target[2][2])
                                elif j is '2':
                                    print("right")
                                    SetTarget(0, target[0][0]+0.50, target[0][1], target[0][2])
                                    SetTarget(1, target[1][0]+0.50, target[1][1], target[1][2])
                                    SetTarget(2, target[2][0]+0.50, target[2][1], target[2][2])
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
                                    circle1 = sequence_1(0,0.40,0.0)
                                    circle2 = sequence_1(1,0.70,90.0)
                                    circle1.start()
                                    circle2.start()
                                    str(input("If you want to stop, Enter any key"))
                                    circle1.stop()
                                    circle2.stop()
                                elif j is '7':
                                    location_debug()
                                elif j is '8':
                                    x = float(input("X : "))
                                    y = float(input("Y : "))
                                    z = float(input("Z : "))
                                    SetTarget(0,x,y,z)
                                elif j is '9':
                                    yaw_target[0] = float(input("input yaw:"))

                                elif j == '10':
                                    rectangle = Rectangle(0,0.40,0.40, 0)
                                    str(input("start"))
                                    rectangle.start()
                                    str(input("If you want to stop, Enter any key"))
                                    rectangle.stop()
                                elif j == '11':
                                    triangle = Triangle(0,0.50,0)
                                    #triangle2 = Triangle(1,0.50,1)
                                    str(input("start"))
                                    triangle.start()
                                    #triangle2.start()
                                    str(input("If you want to stop, Enter any key"))
                                    triangle.stop()
                                    #triangle2.stop()
                                elif j == '12':
                                    #triangle = Triangle(0,0.50,0)

                                    #current position cental
                                    #circle1 = sequence_1(0,0.20,0.0, 1)

                                    circle1 = sequence_1(0,0.50,0.0)
                                    circle2 = sequence_1(1,0.50,45.0)
                                    circle3 = sequence_1(2,0.50,90.0)
                                    circle4 = sequence_1(3,0.50,180.0)
                                    circle5 = sequence_1(4,0.50,225.0)
                                    circle6 = sequence_1(5,0.50,270.0)
                                    #circle4 = sequence_1(3,0.50,270.0)
                                    str(input("start"))
                                    #triangle.start()
                                    circle1.start()
                                    circle2.start()
                                    circle3.start()
                                    circle4.start()
                                    circle5.start()
                                    circle6.start()
                                    #circle4.start()
                                    str(input("If you want to stop, Enter any key"))
                                    #triangle.stop()
                                    circle1.stop()
                                    circle2.stop()
                                    circle3.stop()
                                    circle4.stop()
                                    circle5.stop()
                                    circle6.stop()
                                    #circle4.stop()
                                elif j == '13':
                                    while True:
                                        a = str(input("Enter any key, exit is 'q'"))
                                        if a == 'q':
                                            break
                                        SetTarget(0, pos[1][0], target[0][1], pos[1][2])
                                        SetTarget(1, pos[0][0], target[1][1], pos[0][2])
                                elif j == '14':
                                    P_1()
                                    P_2()

                        elif i is '2':
                            land()
                        elif i is '3':
                            run_chk = 0
                            init_value()
                            for i in range(DroneCnt):
                                pitch[i] = 0
                                roll[i] = 0
                                yaw[i] = 0
                                thrust[i] = 0
                            break

            elif val is 3:
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
                            init_value()
                            #hovering_thread[0].stop()
                            pitch[0] = 0
                            roll[0] = 0
                            yaw[0] = 0
                            thrust[0] = 0
                            #hovering_thread[0] = None
                            break
                    

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

