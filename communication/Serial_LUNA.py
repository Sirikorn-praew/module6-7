import serial
import time
serialPic = serial.Serial(port= 'COM3',timeout = 3, baudrate=115200,
                  xonxoff=0, rtscts=0,bytesize=8, parity='N', stopbits=1)
serialPic.rts = 0
serialPic.dtr = 0
serialPic.timeout = 0.1
serialPic.flush()
listdata = [[100,100,100,0,1],[]]

def toByte(x,y,z,a,g):
    din = [int(x),int(y),int(z),int(a)]
    bytelist = []
    for i in din:
        bytelist.append(i//256)
        bytelist.append(i%256)
    bytelist.append(int(g))
    return bytelist
# Data FF F_ xx xx yy yy zz zz aa aa gg

def Sethome():
    data = [255,241,0,0,0,0,0,0,0,0,0]                            #"FF,F1" + x + y + z + a + g
    #print(data)
    serialPic.write(serial.to_bytes(data))
    t = 0
    while(True):
        a = serialPic.readline()
        if len(a) != 0:
            print(a)
            spic = [b for b in a][0]
            if spic == 241:
                break
        time.sleep(2)
        serialPic.write(serial.to_bytes(data))
        t += 1
        if t >= 2:
            break
    print("LUNA Comimg Home")

def Capture():
    data = [255,242,0,0,0,0,0,0,0,0,0]                            #"FF,F2"
    serialPic.write(serial.to_bytes(data))
    print(data)
    state = 1
    while(True):
        a = serialPic.readline()
        if len(a) != 0:
            print(a)
            spic = [b for b in a][0]
            ac = [255,170,spic,0,0,0,0,0,0,0,0]
            if spic == 5:
                print("state = "+ str(state))
                time.sleep(5)
                serialPic.write(serial.to_bytes(ac))
                print(ac)
                break
            elif spic == state:
                print("state = "+ str(state))
                time.sleep(5)
                serialPic.write(serial.to_bytes(ac))
                print(ac)
                state += 1
            if spic == 99:
                print("error");
                serialPic.write(serial.to_bytes(data))
                state = 1
    print("Capture Finish")

def Catch():
    data = [255,243,0,0,0,0,0,0,0,0,0]                           #"FF,F3" + x + y + z 
    #print(data)
    serialPic.write(serial.to_bytes(data))
    while(True):
        a = serialPic.readline()
        if len(a) != 0:
            print(a)
            spic = [b for b in a][0]
            if spic == 243:
                break
    print("Catch!")

def MoveI(m):
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
        a = input("Angular : ")
        if a == 'q':
            print('quit')
            break
        g = input("Gripper : ")
        if g == 'q':
            print('quit')
            break
        byteL = toByte(x,y,z,a,g)
        if m == "PID":
            data = [255,244,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F4" + x + y + z
        if m == "Traj":
            data = [255,245,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F5" + x + y + z
        print(data)
        serialPic.write(serial.to_bytes(data))
        while(True):
            a = serialPic.readline()
            if len(a) != 0:
                print(a)
                spic = [b for b in a][0]
                if m == "PID":
                    if spic == 244:
                        break
                if m == "Traj":
                    if spic == 245:
                        break    
        print("Move to x="+ x +" , y="+ y + " , z="+ z)
        break

def Move(m,list):
    for i in list:    
        byteL = toByte(list[0],list[1],list[2],list[3],list[4])
        if m == "PID":
            data = [255,244,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F4" + x + y + z
        if m == "Traj":
            data = [255,245,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F5" + x + y + z
        print(data)
        serialPic.write(serial.to_bytes(data))
        while(True):
            a = serialPic.readline()
            if len(a) != 0:
                print(a)
                spic = [b for b in a][0]
                if m == "PID":
                    if spic == 244:
                        break
                if m == "Traj":
                    if spic == 245:
                        break    
        print("Move to x="+ x +" , y="+ y + " , z="+ z)

def Reset():
    data = [255,187,0,0,0,0,0,0,0,0,0]                            #"FF,BB" 
    #print(data)
    serialPic.write(serial.to_bytes(data))
    while(True):
        a = serialPic.readline()
        if len(a) != 0:
            print(a)
            spic = [b for b in a][0]
            if spic == 187:
                time.sleep(1)
                serialPic.rts = 1
                time.sleep(1)
                serialPic.rts = 0
                serialPic.dtr = 0
                serialPic.timeout = 0.1
                serialPic.flush()
                break   
    print("LUNA Reset Complete")

def Mode(m):
    if m == '1':    #Set home
        Sethome()
    elif m == '2':  #Capture
        Capture()
    elif m == '3':  #Catch
        Catch()
    elif m == '4':  #Move
        Move("PID")
    elif m == '5':  #Move
        Move("Traj")
    
def Link():
    print("LUNA Link start!")
    while True:
        print("/////////////////////\nInput Mode You want\n 1: Set home\n 2: Capture\n 3: Catch\n 4: Move PID\n 5: Move Traj\n done: to Out of Link\n/////////////////////")
        m = input("Mode(1/2/3/4/5):")
        if m == 'done':
            serialPic.rts = 1
            print('Out of Link LUNA ;(')
            break
        elif m == 'r':
            Reset()
        elif m == 'rts':
            serialPic.rts = 1
            time.sleep(1)
            serialPic.rts = 0
            print("LUNA Reset-XY Complete")
        elif m in ['1','2','3','4','5']:
            print('Mode: '+m)
            Mode(m)
        else:
            print('Invalid Input')
    
Link()