import serial
import time
# serialArdu = serial.Serial('COM7',115200)
# serialArdu.timeout = 0.1
serialPic = serial.Serial(port= 'COM5',timeout = 3, baudrate=115200,
                  xonxoff=0, rtscts=0,bytesize=8, parity='N', stopbits=1)
serialPic.rts = 0
serialPic.dtr = 0
serialPic.timeout = 0.1
serialPic.flush()
# Data FF F_ xx xx yy yy zz zz aa aa gg
def toByte(x,y):
    x = int(x)
    y = int(y)
    bytelist = []
    if x > 255:
        bytelist.append(1)
        bytelist.append(x-256)
    else:
        bytelist.append(0)
        bytelist.append(x)
    if y > 255:
        bytelist.append(1)
        bytelist.append(y-256)
    else:
        bytelist.append(0)
        bytelist.append(y)
    return bytelist
# def CommuArdu():
#     while True:
#         i = input("input (on/off): ").strip()
#         if i == 'done':
#             print('finished program')
#             break
#         serialArdu.write(i.encode())
#         time.sleep(0.5)
#         print(serialArdu.readline().decode('ascii'))
#     serialArdu.close()
def CommuPic():
    while True:
        x = input("X-axis : ")
        if x == 'q':
            print('quit')
            break
        y = input("Y-axis : ")
        if y == 'q':
            print('quit')
            break
        elif y == 'b':
            print('back')
            continue
        elif  x == 's' and y == 's':
            data = [255,240,0,0,0,0]                                #"FF,F0"
            serialPic.write(serial.to_bytes(data))
            print(data)
            print("Stay")
        elif  x == 'p' and y == 'p':
            data = [255,242,0,0,0,0]                                #"FF,F2"
            serialPic.write(serial.to_bytes(data))
            state = 1
            while(True):
                a = serialPic.readline()
                print(a)
                spic = [b for b in a][0]
                if spic == state:
                    serialPic.write(serial.to_bytes(spic))
                    state += 1
                if spic == 99:
                    serialPic.write(serial.to_bytes(data))
                    state = 1
                if spic == 242:
                    break
            print(data)
            print("Take pic")
        elif  x == 'c' and y == 'c':
            data = [255,243,0,0,1,94]                               #"FF,F3"
            serialPic.write(serial.to_bytes(data))
            print(data)
            print("Catch!")
        elif x == 'h' and y == 'h':
            data = [255,241,0,0,0,0]                                #"FF,F1" + x + y +
            serialPic.write(serial.to_bytes(data))
            print(data)
            print("LUNA Comimg Home")
        else:
            byteL = toByte(x,y)
            data = [255,244,byteL[0],byteL[1],byteL[2],byteL[3]]    #"FF,F4" + x + y
            serialPic.write(serial.to_bytes(data))
            print(data)
            print("Move to x="+ x +" , y="+ y)
        print("Pic :")
        a = serialPic.readline()
        print(str(a))
        print([b for b in a]) 
        time.sleep(0.5)
        # print(serialPic.readline())
        # time.sleep(0.5)
    serialPic.close()

print("LUNA Link start!")
while True:
    s = input("input (XY / Z): ").strip()
    if s == 'done':
        print('finished program')
        break
    # if s == 'Z':
    #     CommuArdu()
    #     continue
    if s == 'XY':
        CommuPic()
        continue
    else:
        print('Invalid Input')
