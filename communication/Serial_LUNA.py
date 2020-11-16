import serial
import time
serialPic = serial.Serial(port= 'COM5',timeout = 3, baudrate=115200,
                  xonxoff=0, rtscts=0,bytesize=8, parity='N', stopbits=1)
serialPic.rts = 0
serialPic.dtr = 0
serialPic.timeout = 0.1
serialPic.flush()

def toByte(x,y,z,a):
    din = [int(x),int(y),int(z),int(a)]
    bytelist = []
    for i in din:
        bytelist.append(i//256)
        bytelist.append(i%256)
    return bytelist
# Data FF F_ xx xx yy yy zz zz aa aa gg
def Sethome():
    data = [255,241,0,0,0,0,0,0,0,0,0]                            #"FF,F1" + x + y + z + a + g
    print(data)
    while(True):
        serialPic.write(serial.to_bytes(data))
        a = serialPic.readline()
        print(a)
        if serialPic.readline() != 0:
            spic = [b for b in a][0]
            if spic == 241:
                break
    print("LUNA Comimg Home")

def Capture():
    data = [255,242,0,0,0,0,0,0,0,0,0]                            #"FF,F2"
    serialPic.write(serial.to_bytes(data))
    state = 1
    while(True):
        a = serialPic.readline()
        print(a)
        spic = [b for b in a][0]
        if spic == state:
            print("state = "+ str(state))
            serialPic.write(serial.to_bytes(spic))
            state += 1
        if spic == 99:
            print("error")
            serialPic.write(serial.to_bytes(data))
            state = 1
        if spic == 242:
            break
    print("Capture Finish")

def Catch():
    data = [255,243,0,0,1,94,0,0,0,0,0]                           #"FF,F3" + x + y + z 
    print(data)
    # serialPic.write(serial.to_bytes(data))
    while(True):
        serialPic.write(serial.to_bytes(data))
        a = serialPic.readline()
        print(a)
        spic = [b for b in a][0]
        if spic == 243:
            break
    print("Catch!")

def Move():
    while True:
        x = input("X-axis : ")
        if x == 'q':
            print('quit')
            break
        y = input("Y-axis : ")
        if y == 'q':
            print('quit')
            break
        z = input("Z-axis : ")
        if z == 'q':
            print('quit')
            break
        byteL = toByte(x,y,z,0)
        byteL.append(0)
        data = [255,244,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F4" + x + y + z
        print(data)
        serialPic.write(serial.to_bytes(data))
        while(True):
            serialPic.write(serial.to_bytes(data))
            a = serialPic.readline()
            print(a)
            spic = [b for b in a][0]
            if spic == 244:
                break
        print("Move to x="+ x +" , y="+ y + " , z="+ z)
        break

def Reset():
    data = [255,187,0,0,0,0,0,0,0,0,0]                            #"FF,BB" 
    print(data)
    serialPic.write(serial.to_bytes(data))
    while(True):
        a = serialPic.readline()
        print(a)
        spic = [b for b in a][0]
        if spic == 187:
            serialPic.rts = 1
            time.sleep(1)
            serialPic.rts = 0
    print("LUNA Reset Complete")

def Mode(m):
    if m == '1':    #Set home
        Sethome()
    elif m == '2':  #Capture
        Capture()
    elif m == '3':  #Catch
        Catch()
    elif m == '4':  #Move
        Move()
    
def Link():
    print("LUNA Link start!")
    while True:
        print("/////////////////////\nInput Mode You want\n 1: Set home\n 2: Capture\n 3: Catch\n 4: Move\n done: to Out of Link\n/////////////////////")
        m = input("Mode(1/2/3/4):")
        if m == 'done':
            serialPic.rts = 1
            print('Out of Link LUNA ;(')
            break
        elif m == 'r':
            Reset()
        elif m in ['1','2','3','4']:
            print('Mode: '+m)
            Mode(m)
        else:
            print('Invalid Input')
    
Link()