import client
import sys
from threading import Thread
import time

connect_state = [0]*10

thrust = 0
pitch = 0.0

class thread(Thread):
    def __init__(self,channel, *args):
        super(thread, self).__init__(*args)
        global thrust
        global pitch
        self.channel = channel
    def run(self):
        while True:
            #print(thrust)
            client.setControl(self.channel,0.0,pitch,0.0,thrust)
            time.sleep(0.01)

print("If you don't know command, Enther the 'help'")
while True:
    sel = str(input("<<"))

    if sel == "connect" :
        print("Channel input :")
        channel = str(input())
        client.Connect(channel)
        connect_state[0] = 1
    elif sel == "connect all" :
        #client.first_Connect()
        #client.second_Connect()
        client.all_Connect()
        connect_state[0:10] = [1,1,1,1,1,1,1,1,1,1]
    elif sel == "disconnect all" :
        client.first_Disconnect()
        client.second_Disconnect()
        connect_state[0:10] = [0,0,0,0,0,0,0,0,0,0]
    elif sel == "log" :
        print("Channel input :")
        channel = str(input())
        if connect_state[0] is 1:
            client.setLog(channel)
        else:
            print("First, connect drone!!!")
    elif sel == "control":
        print("Channel input :")
        channel = str(input())
        client.setControl(channel,0.0,0.0,0.0,0)
        t = thread(channel)
        t.start()
        global thrust
        global pitch
        while(1):
            print("1. input mode 2. auto mode 3. pitch control mode")
            val = int(input())
            if val is 1 :
                print("thrust input:")
                thrust = int(input())
                if thrust == -1 :
                    t.stop()
                    break
            if val is 2 :
                thrust = 2000
                time.sleep(2)
                pitch = 1.0
                time.sleep(1)
                pitch = 0.0
                time.sleep(1)
                thrust = 0
            if val is 3 :
                print("thrust input:")
                thrust = int(input())
                for i in range(5):
                    print("pitch input:")
                    pitch = float(input())
                thrust = 0
                pitch = 0

    elif sel == "exit" :
        sys.exit(0)
    elif sel == "help" :
        print("command : 1. connect 2. connect all 3. disconnect 4. disconnect all 5. log")
        print("1. connect => Connect selected a drone ex) connect 1")
        print("connect 1 <- Drone number you want to connect")
        print("2. connect all => At a time, Connect all drone")
        print("3. disconnect => disconnect selected a drone ex)disconnect 1")
        print("4. disconnect all => At a time, Disconnect all drone")
        print("5. log => Show roll, pitch data 10 times")
        print("If you want to exit, Enter the 'exit'")
    else :
        print("Error : unvalid command")

