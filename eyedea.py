dongleCnt = 12

def find_Unused_Dongle():
    f = open("info", "r")
    data = f.read()
    a = [0]*dongleCnt
    idx = 0
    unused = 0

    chk = False
    for c in data :
        if c != '\n' :
            if c == '0':
                if chk is False :
                    unused = idx
                    chk = True
                a[idx] = 0
                idx+=1
                if idx is dongleCnt:
                    break
            elif c == '1':
                a[idx] = 1
                idx+=1
                if idx is dongleCnt:
                    break
    f.close()

    a[unused] = 1

    f = open("info", "w")
    f.write(str(a))
    f.close()

    return unused

def release_dongle(num = 0):
    f = open("info", "r")
    data = f.read()
    a = [0]*dongleCnt
    idx = 0
    unused = 0

    for c in data :
        if c != '\n' :
            if c == '0':
                a[idx] = 0
                idx+=1
            elif c == '1':
                a[idx] = 1
                idx+=1
    f.close()

    f = open("info", "w")
    a[num] = 0
    f.write(str(a))
    f.close()

