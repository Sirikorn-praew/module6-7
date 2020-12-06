import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QApplication

# import serial
# import time
# serialPic = serial.Serial(port= 'COM5',timeout = 3, baudrate=115200,
#                   xonxoff=0, rtscts=0,bytesize=8, parity='N', stopbits=1)
# serialPic.rts = 0
# serialPic.dtr = 0
# serialPic.timeout = 0.1
# serialPic.flush()

def toByte(x,y,z,a):
    din = [int(x),int(y),int(z),int(a)]
    bytelist = []
    for i in din:
        bytelist.append(i//256)
        bytelist.append(i%256)
    return bytelist
# Data FF F_ xx xx yy yy zz zz aa aa gg

class Ui_Main(object):
    def setupUi(self, Main):
        Main.setObjectName("Main")
        Main.resize(1000, 700)
        Main.setStyleSheet("background-color: rgb(48, 48, 48);")
        self.Mode = QtWidgets.QLabel(Main)
        self.Mode.setGeometry(QtCore.QRect(40, 40, 250, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(30)
        font.setBold(True)
        font.setWeight(75)
        self.Mode.setFont(font)
        self.Mode.setStyleSheet("color: rgb(255, 255, 255);")
        self.Mode.setObjectName("Mode")
        self.sethomeB = QtWidgets.QPushButton(Main)
        self.sethomeB.setGeometry(QtCore.QRect(50, 120, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.sethomeB.setFont(font)
        self.sethomeB.setMouseTracking(False)
        self.sethomeB.setStyleSheet("background-color: rgb(0, 85, 127);\n"
"color: rgb(255, 255, 255);")
        self.sethomeB.setCheckable(False)
        self.sethomeB.setObjectName("sethomeB")
        self.sethomeB.clicked.connect(Sethome)
        self.captureB = QtWidgets.QPushButton(Main)
        self.captureB.setGeometry(QtCore.QRect(50, 200, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.captureB.setFont(font)
        self.captureB.setMouseTracking(False)
        self.captureB.setStyleSheet("background-color: rgb(255, 168, 16);\n"
"color: rgb(255, 255, 255);")
        self.captureB.setCheckable(False)
        self.captureB.setObjectName("captureB")
        self.captureB.clicked.connect(Capture)
        self.catchB = QtWidgets.QPushButton(Main)
        self.catchB.setGeometry(QtCore.QRect(50, 280, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.catchB.setFont(font)
        self.catchB.setMouseTracking(False)
        self.catchB.setStyleSheet("background-color: rgb(255, 100, 120);\n"
"color: rgb(255, 255, 255);")
        self.catchB.setCheckable(False)
        self.catchB.setObjectName("catchB")
        self.catchB.clicked.connect(Catch)
        self.move_pidB = QtWidgets.QPushButton(Main)
        self.move_pidB.setGeometry(QtCore.QRect(50, 360, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.move_pidB.setFont(font)
        self.move_pidB.setMouseTracking(False)
        self.move_pidB.setStyleSheet("background-color: rgb(69, 138, 207);\n"
"color: rgb(255, 255, 255);")
        self.move_pidB.setCheckable(False)
        self.move_pidB.setObjectName("move_pidB")
        # self.move_pidB.clicked.connect(Move("PID"))
        self.move_trajB = QtWidgets.QPushButton(Main)
        self.move_trajB.setGeometry(QtCore.QRect(50, 440, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.move_trajB.setFont(font)
        self.move_trajB.setMouseTracking(False)
        self.move_trajB.setStyleSheet("background-color: rgb(9, 143, 116);\n"
"color: rgb(255, 255, 255);")
        self.move_trajB.setCheckable(False)
        self.move_trajB.setObjectName("move_trajB")
        self.resetB = QtWidgets.QPushButton(Main)
        self.resetB.setGeometry(QtCore.QRect(330, 40, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.resetB.setFont(font)
        self.resetB.setMouseTracking(False)
        self.resetB.setStyleSheet("background-color: rgb(201, 0, 0);\n"
"color: rgb(255, 255, 255);")
        self.resetB.setCheckable(False)
        self.resetB.setObjectName("resetB")
        self.resetB.clicked.connect(Reset)
        self.sent_pidB = QtWidgets.QPushButton(Main)
        self.sent_pidB.setGeometry(QtCore.QRect(770, 365, 100, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.sent_pidB.setFont(font)
        self.sent_pidB.setMouseTracking(False)
        self.sent_pidB.setStyleSheet("background-color: rgb(12, 169, 51);\n"
"color: rgb(255, 255, 255);")
        self.sent_pidB.setCheckable(False)
        self.sent_pidB.setObjectName("sent_pidB")
        self.sent_pidB.clicked.connect(self.insertTextPID)
        self.sent_trajB = QtWidgets.QPushButton(Main)
        self.sent_trajB.setGeometry(QtCore.QRect(770, 445, 100, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.sent_trajB.setFont(font)
        self.sent_trajB.setMouseTracking(False)
        self.sent_trajB.setStyleSheet("background-color: rgb(12, 169, 51);\n"
"color: rgb(255, 255, 255);")
        self.sent_trajB.setCheckable(False)
        self.sent_trajB.setObjectName("sent_trajB")
        self.sent_trajB.clicked.connect(self.insertTextTraj)
        self.InX = QtWidgets.QLineEdit(Main)
        self.InX.setGeometry(QtCore.QRect(320, 365, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InX.setFont(font)
        self.InX.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InX.setObjectName("InX")
        self.labelX = QtWidgets.QLabel(Main)
        self.labelX.setGeometry(QtCore.QRect(280, 365, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelX.setFont(font)
        self.labelX.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelX.setObjectName("labelX")
        self.InY = QtWidgets.QLineEdit(Main)
        self.InY.setGeometry(QtCore.QRect(440, 365, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InY.setFont(font)
        self.InY.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InY.setObjectName("InY")
        self.labelY = QtWidgets.QLabel(Main)
        self.labelY.setGeometry(QtCore.QRect(400, 365, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelY.setFont(font)
        self.labelY.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelY.setObjectName("labelY")
        self.textEdit = QtWidgets.QTextEdit(Main)
        self.textEdit.setGeometry(QtCore.QRect(300, 530, 400, 100))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.textEdit.setFont(font)
        self.textEdit.setStyleSheet("color: rgb(255, 255, 255);")
        self.textEdit.setObjectName("textEdit")
        self.InZ = QtWidgets.QLineEdit(Main)
        self.InZ.setGeometry(QtCore.QRect(560, 365, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InZ.setFont(font)
        self.InZ.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InZ.setObjectName("InZ")
        self.labelZ = QtWidgets.QLabel(Main)
        self.labelZ.setGeometry(QtCore.QRect(520, 365, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelZ.setFont(font)
        self.labelZ.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelZ.setObjectName("labelZ")
        self.InA = QtWidgets.QLineEdit(Main)
        self.InA.setGeometry(QtCore.QRect(680, 365, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InA.setFont(font)
        self.InA.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InA.setObjectName("InX")
        self.labelA = QtWidgets.QLabel(Main)
        self.labelA.setGeometry(QtCore.QRect(640, 365, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelA.setFont(font)
        self.labelA.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelA.setObjectName("labelA")
        self.InX_2 = QtWidgets.QLineEdit(Main)
        self.InX_2.setGeometry(QtCore.QRect(320, 445, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InX_2.setFont(font)
        self.InX_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InX_2.setObjectName("InX_2")
        self.InY_2 = QtWidgets.QLineEdit(Main)
        self.InY_2.setGeometry(QtCore.QRect(440, 445, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InY_2.setFont(font)
        self.InY_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InY_2.setObjectName("InY_2")
        self.labelY_2 = QtWidgets.QLabel(Main)
        self.labelY_2.setGeometry(QtCore.QRect(400, 445, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelY_2.setFont(font)
        self.labelY_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelY_2.setObjectName("labelY_2")
        self.InZ_2 = QtWidgets.QLineEdit(Main)
        self.InZ_2.setGeometry(QtCore.QRect(560, 445, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InZ_2.setFont(font)
        self.InZ_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InZ_2.setObjectName("InZ_2")
        self.InA_2 = QtWidgets.QLineEdit(Main)
        self.InA_2.setGeometry(QtCore.QRect(680, 445, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InA_2.setFont(font)
        self.InA_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InA_2.setObjectName("InA_2")
        self.labelX_2 = QtWidgets.QLabel(Main)
        self.labelX_2.setGeometry(QtCore.QRect(280, 445, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelX_2.setFont(font)
        self.labelX_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelX_2.setObjectName("labelX_2")
        self.labelZ_2 = QtWidgets.QLabel(Main)
        self.labelZ_2.setGeometry(QtCore.QRect(520, 445, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelZ_2.setFont(font)
        self.labelZ_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelZ_2.setObjectName("labelZ_2")
        self.labelA_2 = QtWidgets.QLabel(Main)
        self.labelA_2.setGeometry(QtCore.QRect(640, 445, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelA_2.setFont(font)
        self.labelA_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelA_2.setObjectName("labelA_2")

        self.retranslateUi(Main)
        QtCore.QMetaObject.connectSlotsByName(Main)

    def retranslateUi(self, Main):
        _translate = QtCore.QCoreApplication.translate
        Main.setWindowTitle(_translate("Main", "Form"))
        self.Mode.setText(_translate("Main", "LUNA Mode"))
        self.sethomeB.setText(_translate("Main", "Set home"))
        self.captureB.setText(_translate("Main", "Capture"))
        self.catchB.setText(_translate("Main", "Catch"))
        self.move_pidB.setText(_translate("Main", "Move PID"))
        self.move_trajB.setText(_translate("Main", "Move Traj"))
        self.resetB.setText(_translate("Main", "Reset"))
        self.sent_pidB.setText(_translate("Main", "Sent"))
        self.sent_trajB.setText(_translate("Main", "Sent"))
        self.InX.setText(_translate("Main", "0"))
        self.InY.setText(_translate("Main", "0"))
        self.InZ.setText(_translate("Main", "0"))
        self.InA.setText(_translate("Main", "0"))
        self.InX_2.setText(_translate("Main", "0"))
        self.InY_2.setText(_translate("Main", "0"))
        self.InZ_2.setText(_translate("Main", "0"))
        self.InA_2.setText(_translate("Main", "0"))
        self.labelX.setText(_translate("Main", "X:"))
        self.labelY.setText(_translate("Main", "Y:"))
        self.labelZ.setText(_translate("Main", "Z:"))
        self.labelA.setText(_translate("Main", "A:"))
        self.labelY_2.setText(_translate("Main", "Y:"))
        self.labelX_2.setText(_translate("Main", "X:"))
        self.labelZ_2.setText(_translate("Main", "Z:"))
        self.labelA_2.setText(_translate("Main", "A:"))

    def insertTextPID(self):
        tx = self.InX.text()
        ty = self.InY.text()
        tz = self.InZ.text()
        ta = self.InA.text()
        Move("PID",tx,ty,tz,ta)
    def insertTextTraj(self):
        tx = self.InX_2.text()
        ty = self.InY_2.text()
        tz = self.InZ_2.text()
        ta = self.InA_2.text()
        Move("Traj",tx,ty,tz,ta)
        # print(tx+', '+ty+', '+tz)


def Sethome():
    data = [255,241,0,0,0,0,0,0,0,0,0]                            #"FF,F1" + x + y + z + a + g
    print(data)
    # serialPic.write(serial.to_bytes(data))
    # t = 0
    # while(True):
    #     a = serialPic.readline()
    #     if len(a) != 0:
    #         print(a)
    #         spic = [b for b in a][0]
    #         if spic == 241:
    #             break
    #     time.sleep(2)
    #     serialPic.write(serial.to_bytes(data))
    #     t += 1
    #     if t >= 2:
    #         break
    print("LUNA Comimg Home")
    t = "LUNA Comimg Home"
    mySW.SetMsg(t)

def Capture():
    data = [255,242,0,0,0,0,0,0,0,0,0]                            #"FF,F2"
    print(data)
    # serialPic.write(serial.to_bytes(data))
    # state = 1
    # while(True):
    #     a = serialPic.readline()
    #     print(a)
    #     spic = [b for b in a][0]
    #     if spic == state:
    #         print("state = "+ str(state))
    #         serialPic.write(serial.to_bytes(spic))
    #         state += 1
    #     if spic == 99:
    #         print("error");
    #         serialPic.write(serial.to_bytes(data))
    #         state = 1
    #     if spic == 242:
    #         break
    print("Capture Finish")
    t = "Capture Finish"
    mySW.SetMsg(t)

def Catch():
    data = [255,243,0,0,0,0,0,0,0,0,0]                           #"FF,F3" + x + y + z 
    print(data)
    # serialPic.write(serial.to_bytes(data))
    # while(True):
    #     a = serialPic.readline()
    #     if len(a) != 0:
    #         print(a)
    #         spic = [b for b in a][0]
    #         if spic == 243:
    #             break
    print("Catch!")
    t = "Catch!"
    mySW.SetMsg(t)


def Move(m,x,y,z,a):
    byteL = toByte(x,y,z,a)
    byteL.append(0)
    if m == "PID":
        data = [255,244,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F4" + x + y + z
    if m == "Traj":
        data = [255,245,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F5" + x + y + z
    print(data)
    # serialPic.write(serial.to_bytes(data))
    # while(True):
    #     a = serialPic.readline()
    #     if len(a) != 0:
    #         print(a)
    #         spic = [b for b in a][0]
    #         if m == "PID":
    #             if spic == 244:
    #                 break
    #         if m == "Traj":
    #             if spic == 245:
    #                 break    
    print("Move to x="+ x +" , y="+ y + " , z="+ z)
    t = "Move to x="+ x +" , y="+ y + " , z="+ z + " , a="+ a
    mySW.SetMsg(t)
    

def Reset():
    data = [255,187,0,0,0,0,0,0,0,0,0]                            #"FF,BB" 
    print(data)
    # serialPic.write(serial.to_bytes(data))
    # while(True):
    #     a = serialPic.readline()
    #     if len(a) != 0:
    #         print(a)
    #         spic = [b for b in a][0]
    #         if spic == 187:
    #             time.sleep(1)
    #             serialPic.rts = 1
    #             time.sleep(1)
    #             serialPic.rts = 0
    #             serialPic.dtr = 0
    #             serialPic.timeout = 0.1
    #             serialPic.flush()
    #             break   
    print("LUNA Reset Complete")
    t = "LUNA Reset Complete"
    mySW.SetMsg(t)

class ControlMainWindow(QMainWindow): #เพิ่มคลาสมาอีก
        def __init__(self, parent=None):
                super(ControlMainWindow, self).__init__(parent)
                self.ui =  Ui_Main() #ดึงคลาส  Ui_Form ที่เก็บรายละอียด ui มาใช้
                self.ui.setupUi(self)
        def SetMsg(self,t):
            self.ui.textEdit.setText(t)
   
if __name__ == "__main__":
    app = QApplication(sys.argv)
    mySW = ControlMainWindow() 
    mySW.show()  #แสดง
    sys.exit(app.exec_())
def main():
        app = QApplication(sys.argv)
        mySW = ControlMainWindow()  #ดึงคลาส ControlMainWindow มาใช้
        mySW.show()
        sys.exit(app.exec_())
if __name__ == "__main__":
  a = main() 