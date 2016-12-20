
def ServerAddr(kind="", idx="") :
    if kind is "cmd":
        return "tcp://localhost:20"+idx+"0"
    elif kind is "log":
        return "tcp://localhost:20"+idx+"1"
    elif kind is "ctrl":
        return "tcp://localhost:20"+idx+"4"

def connDrone(channel=""):
    if channel == '17':
        uri = "radio://0/"+channel+"/1M/1717171717"
    elif channel == '18':
        uri = "radio://0/"+channel+"/2M/ABABABABAB"
    elif channel == '42':
        uri = "radio://0/"+channel+"/2M/4242424242"
    elif channel == '54':
        uri = "radio://0/"+channel+"/2M/E7E7E7E7E7"
    elif channel == '3':
        uri = "radio://0/"+channel+"/1M/E7E7E7E7E7"
    elif channel == '125':
        uri = "radio://0/"+channel+"/2M/E7E7E7E7E7"
    elif channel == '64':
        uri = "radio://0/"+channel+"/250K/E7E7E7E7E7"
    elif channel == '99':
        uri = "radio://0/"+channel+"/2M/E7E7E7E7E7"
    elif channel == '30':
        uri = "radio://0/"+channel+"/2M/E7E7E7E7E7"
    elif channel == '87':
        uri = "radio://0/"+channel+"/1M/E7E7E7E7E7"
    else:
        uri = "radio://0/"+channel+"/2M/E7E7E7E7E7"
    send = {
		"version": 1,
		"cmd" : "connect",
		"uri" : uri
	}
    return send

def disconnDrone(channel=""):
    uri = "radio://0/"+channel+"/250K"
    send = {
        "version": 1,
        "cmd" : "disconnect",
        "uri" : uri
    }
    return send

def controlJson(roll=0.0,pitch=0.0,yaw=0.0,thrust=0):
    send = {
            "version": 1,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "thrust": 0.0
        }
    send["roll"] = roll
    send["pitch"] = pitch
    send["yaw"] = yaw
    send["thrust"] = thrust
#print("json thrust " + str(thrust))
#print(send)
    return send

def controlParam(name="", value=0.0):
    param = {
        "version": 1,
        "cmd" : "param",
        "name" : name,
        "value" : value
    }
    return param
 
log_create = {
    "version" : 1,
    "cmd" : "log",
    "action" : "create",
    "name" : "stabilizer block",
    "period" : 100,
    "variables" : [
        #"pm.vbat",
        #"pm.state",
        #"stabilizer.roll",
        #"stabilizer.pitch",
        #"stabilizer.yaw",
        #"stabilizer.thrust",
        "gyro.z",
        "acc.WZ"
    ]
}

log_start = {
    "version": 1,
    "cmd": "log",
    "action": "start",
    "name": "stabilizer block"
}

log_stop = {
    "version": 1,
    "cmd" : "log",
    "action" : "stop",
    "name" : "stabilizer block"
}

log_delete = {
    "version": 1,
    "cmd" : "log",
    "action" : "delete",
    "name" : "stabilizer block"
}


