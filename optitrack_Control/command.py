
def ServerAddr(kind="", idx="") :
    if kind is "cmd":
        return "tcp://localhost:20"+idx+"0"
    elif kind is "log":
        return "tcp://localhost:20"+idx+"1"
    elif kind is "ctrl":
        return "tcp://localhost:20"+idx+"4"

def connDrone(channel=""):
    uri = "radio://0/"+channel+"/2M"
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
        "acc.z"
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


