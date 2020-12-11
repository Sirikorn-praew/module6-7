# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'LUNA.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QPixmap

import cv2
import os

import serial
import time
serialPic = serial.Serial(port= 'COM3',timeout = 3, baudrate=115200,
                  xonxoff=0, rtscts=0,bytesize=8, parity='N', stopbits=1)
serialPic.rts = 0
serialPic.dtr = 0
serialPic.timeout = 0.1
serialPic.flush()

st_Sethome = 0
end = 0
listdata = [[100,100,100,0,1],[300,100,200,0,1],[300,100,200,135,1],[100,300,100,135,1]]

def toByte(x,y,z,a,g):
    din = [int(x),int(y),int(z),int(a)]
    bytelist = []
    for i in din:
        bytelist.append(i//256)
        bytelist.append(i%256)
    bytelist.append(int(g))
    return bytelist
# Data FF F_ xx xx yy yy zz zz aa aa gg

class Ui_Main(object):
    def setupUi(self, Main):
        Main.setObjectName("Main")
        Main.resize(1280, 960)
        Main.setStyleSheet("background-color: rgb(48, 48, 48);")
        self.sethomeB = QtWidgets.QPushButton(Main)
        self.sethomeB.setGeometry(QtCore.QRect(45, 380, 200, 50))
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
        self.captureB = QtWidgets.QPushButton(Main)
        self.captureB.setGeometry(QtCore.QRect(45, 460, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.captureB.setFont(font)
        self.captureB.setMouseTracking(False)
        self.captureB.setStyleSheet("background-color: rgb(255, 182, 3);\n"
"color: rgb(255, 255, 255);")
        self.captureB.setCheckable(False)
        self.captureB.setObjectName("captureB")
        self.catchB = QtWidgets.QPushButton(Main)
        self.catchB.setGeometry(QtCore.QRect(45, 540, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.catchB.setFont(font)
        self.catchB.setMouseTracking(False)
        self.catchB.setStyleSheet("background-color: rgb(242, 103, 68);\n"
"color: rgb(255, 255, 255);")
        self.catchB.setCheckable(False)
        self.catchB.setObjectName("catchB")
        self.move_pidB = QtWidgets.QPushButton(Main)
        self.move_pidB.setGeometry(QtCore.QRect(45, 620, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.move_pidB.setFont(font)
        self.move_pidB.setMouseTracking(False)
        self.move_pidB.setStyleSheet("background-color: rgb(234, 93, 92);\n"
"color: rgb(255, 255, 255);")
        self.move_pidB.setCheckable(False)
        self.move_pidB.setObjectName("move_pidB")
        self.move_trajB = QtWidgets.QPushButton(Main)
        self.move_trajB.setGeometry(QtCore.QRect(45, 700, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.move_trajB.setFont(font)
        self.move_trajB.setMouseTracking(False)
        self.move_trajB.setStyleSheet("background-color: rgb(93, 191, 178);\n"
"color: rgb(255, 255, 255);")
        self.move_trajB.setCheckable(False)
        self.move_trajB.setObjectName("move_trajB")
        self.resetB = QtWidgets.QPushButton(Main)
        self.resetB.setGeometry(QtCore.QRect(1050, 40, 200, 50))
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

        self.quitB = QtWidgets.QPushButton(Main)
        self.quitB.setGeometry(QtCore.QRect(1080, 550, 100, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.quitB.setFont(font)
        self.quitB.setMouseTracking(False)
        self.quitB.setStyleSheet("background-color: rgb(201, 0, 0);\n"
"color: rgb(255, 255, 255);")
        self.quitB.setCheckable(False)
        self.quitB.setObjectName("quitB")

        self.sent_pidB = QtWidgets.QPushButton(Main)
        self.sent_pidB.setGeometry(QtCore.QRect(1080, 625, 100, 40))
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
        self.sent_trajB = QtWidgets.QPushButton(Main)
        self.sent_trajB.setGeometry(QtCore.QRect(1080, 705, 100, 40))
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
        self.InX = QtWidgets.QLineEdit(Main)
        self.InX.setGeometry(QtCore.QRect(520, 625, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InX.setFont(font)
        self.InX.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InX.setObjectName("InX")
        self.labelX = QtWidgets.QLabel(Main)
        self.labelX.setGeometry(QtCore.QRect(480, 625, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelX.setFont(font)
        self.labelX.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelX.setObjectName("labelX")
        self.InY = QtWidgets.QLineEdit(Main)
        self.InY.setGeometry(QtCore.QRect(640, 625, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InY.setFont(font)
        self.InY.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InY.setObjectName("InY")
        self.labelY = QtWidgets.QLabel(Main)
        self.labelY.setGeometry(QtCore.QRect(600, 625, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelY.setFont(font)
        self.labelY.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelY.setObjectName("labelY")
        self.textEdit = QtWidgets.QTextEdit(Main)
        self.textEdit.setGeometry(QtCore.QRect(550, 790, 400, 100))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.textEdit.setFont(font)
        self.textEdit.setStyleSheet("color: rgb(255, 255, 255);")
        self.textEdit.setObjectName("textEdit")
        self.InZ = QtWidgets.QLineEdit(Main)
        self.InZ.setGeometry(QtCore.QRect(760, 625, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InZ.setFont(font)
        self.InZ.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InZ.setObjectName("InZ")
        self.labelZ = QtWidgets.QLabel(Main)
        self.labelZ.setGeometry(QtCore.QRect(720, 625, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelZ.setFont(font)
        self.labelZ.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelZ.setObjectName("labelZ")
        self.InX_2 = QtWidgets.QLineEdit(Main)
        self.InX_2.setGeometry(QtCore.QRect(520, 705, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InX_2.setFont(font)
        self.InX_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InX_2.setObjectName("InX_2")
        self.InY_2 = QtWidgets.QLineEdit(Main)
        self.InY_2.setGeometry(QtCore.QRect(640, 705, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InY_2.setFont(font)
        self.InY_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InY_2.setObjectName("InY_2")
        self.labelY_2 = QtWidgets.QLabel(Main)
        self.labelY_2.setGeometry(QtCore.QRect(600, 705, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelY_2.setFont(font)
        self.labelY_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelY_2.setObjectName("labelY_2")
        self.InZ_2 = QtWidgets.QLineEdit(Main)
        self.InZ_2.setGeometry(QtCore.QRect(760, 705, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InZ_2.setFont(font)
        self.InZ_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InZ_2.setObjectName("InZ_2")
        self.labelX_2 = QtWidgets.QLabel(Main)
        self.labelX_2.setGeometry(QtCore.QRect(480, 705, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelX_2.setFont(font)
        self.labelX_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelX_2.setObjectName("labelX_2")
        self.labelZ_2 = QtWidgets.QLabel(Main)
        self.labelZ_2.setGeometry(QtCore.QRect(720, 705, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelZ_2.setFont(font)
        self.labelZ_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelZ_2.setObjectName("labelZ_2")
        self.labelA = QtWidgets.QLabel(Main)
        self.labelA.setGeometry(QtCore.QRect(840, 625, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelA.setFont(font)
        self.labelA.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelA.setObjectName("labelA")
        self.InA = QtWidgets.QLineEdit(Main)
        self.InA.setGeometry(QtCore.QRect(880, 625, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InA.setFont(font)
        self.InA.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InA.setObjectName("InA")
        self.InA_2 = QtWidgets.QLineEdit(Main)
        self.InA_2.setGeometry(QtCore.QRect(880, 705, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InA_2.setFont(font)
        self.InA_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InA_2.setObjectName("InA_2")
        self.labelA_2 = QtWidgets.QLabel(Main)
        self.labelA_2.setGeometry(QtCore.QRect(840, 705, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelA_2.setFont(font)
        self.labelA_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelA_2.setObjectName("labelA_2")
        self.frame = QtWidgets.QFrame(Main)
        self.frame.setGeometry(QtCore.QRect(0, 0, 300, 960))
        self.frame.setStyleSheet("background-color: rgb(31, 38, 107);")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.labelPic_2 = QtWidgets.QLabel(self.frame)
        self.labelPic_2.setGeometry(QtCore.QRect(0, 0, 300, 960))
        self.labelPic_2.setObjectName("labelPic_2")

        pixmap = QPixmap('L_G16.jpg')    
        self.labelPic_2.setPixmap(pixmap)

        self.labelPic = QtWidgets.QLabel(Main)
        self.labelPic.setGeometry(QtCore.QRect(500, 90, 500, 500))
        self.labelPic.setObjectName("labelPic")

        pixmap = QPixmap('Realy.jpg')
        pixmap = pixmap.scaled(500,500)      
        self.labelPic.setPixmap(pixmap)

        self.comboBox = QtWidgets.QComboBox(Main)
        self.comboBox.setGeometry(QtCore.QRect(1000, 625, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.comboBox.setFont(font)
        self.comboBox.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.labelA_3 = QtWidgets.QLabel(Main)
        self.labelA_3.setGeometry(QtCore.QRect(960, 625, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelA_3.setFont(font)
        self.labelA_3.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelA_3.setObjectName("labelA_3")
        self.labelA_4 = QtWidgets.QLabel(Main)
        self.labelA_4.setGeometry(QtCore.QRect(960, 705, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelA_4.setFont(font)
        self.labelA_4.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelA_4.setObjectName("labelA_4")
        self.comboBox_2 = QtWidgets.QComboBox(Main)
        self.comboBox_2.setGeometry(QtCore.QRect(1000, 705, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.comboBox_2.setFont(font)
        self.comboBox_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.comboBox_2.setObjectName("comboBox_2")
        self.comboBox_2.addItem("")
        self.comboBox_2.addItem("")
        self.comboBox_2.setItemIcon
        self.sethomeB.clicked.connect(Sethome)
        self.captureB.clicked.connect(Capture)
        self.catchB.clicked.connect(Catch)
        self.resetB.clicked.connect(Reset) 
        self.quitB.clicked.connect(Quit)
        self.sent_pidB.clicked.connect(self.insertTextPID)
        self.sent_trajB.clicked.connect(self.insertTextTraj)
        self.move_pidB.clicked.connect(self.connectPID)
        self.move_trajB.clicked.connect(self.connectTraj)
        self.frame.raise_()
        self.sethomeB.raise_()
        self.captureB.raise_()
        self.catchB.raise_()
        self.move_pidB.raise_()
        self.move_trajB.raise_()
        self.resetB.raise_()
        self.quitB.raise_()
        self.sent_pidB.raise_()
        self.sent_trajB.raise_()
        self.InX.raise_()
        self.labelX.raise_()
        self.InY.raise_()
        self.labelY.raise_()
        self.textEdit.raise_()
        self.InZ.raise_()
        self.labelZ.raise_()
        self.InX_2.raise_()
        self.InY_2.raise_()
        self.labelY_2.raise_()
        self.InZ_2.raise_()
        self.labelX_2.raise_()
        self.labelZ_2.raise_()
        self.labelA.raise_()
        self.InA.raise_()
        self.InA_2.raise_()
        self.labelA_2.raise_()
        self.labelPic.raise_()
        self.comboBox.raise_()
        self.labelA_3.raise_()
        self.labelA_4.raise_()
        self.comboBox_2.raise_()

        self.retranslateUi(Main)
        QtCore.QMetaObject.connectSlotsByName(Main)

    def retranslateUi(self, Main):
        _translate = QtCore.QCoreApplication.translate
        Main.setWindowTitle(_translate("Main", "Form"))
        self.sethomeB.setText(_translate("Main", "Set home"))
        self.captureB.setText(_translate("Main", "Capture"))
        self.catchB.setText(_translate("Main", "Catch"))
        self.move_pidB.setText(_translate("Main", "Move PID"))
        self.move_trajB.setText(_translate("Main", "Move Traj"))
        self.resetB.setText(_translate("Main", "Reset"))
        self.quitB.setText(_translate("Main", "quit"))
        self.sent_pidB.setText(_translate("Main", "Sent"))
        self.sent_trajB.setText(_translate("Main", "Sent"))
        self.labelX.setText(_translate("Main", "X:"))
        self.labelY.setText(_translate("Main", "Y:"))
        self.labelZ.setText(_translate("Main", "Z:"))
        self.labelY_2.setText(_translate("Main", "Y:"))
        self.labelX_2.setText(_translate("Main", "X:"))
        self.labelZ_2.setText(_translate("Main", "Z:"))
        self.labelA.setText(_translate("Main", "A:"))
        self.labelA_2.setText(_translate("Main", "A:"))
        #self.labelPic_2.setText(_translate("Main", "<html><head/><body><p><img src=\":/newPrefix/128038538_288306339279960_6746964841779541107_n.jpg\"/></p></body></html>"))
        #self.labelPic.setText(_translate("Main", "<html><head/><body><p><img src=\":/newPrefix/Real5.png\"/></p></body></html>"))
        self.comboBox.setItemText(0, _translate("Main", "0"))
        self.comboBox.setItemText(1, _translate("Main", "255"))
        self.labelA_3.setText(_translate("Main", "G:"))
        self.labelA_4.setText(_translate("Main", "G:"))
        self.comboBox_2.setItemText(0, _translate("Main", "0"))
        self.comboBox_2.setItemText(1, _translate("Main", "255"))
        self.InX.setText(_translate("Main", "0"))
        self.InY.setText(_translate("Main", "0"))
        self.InZ.setText(_translate("Main", "0"))
        self.InA.setText(_translate("Main", "0"))
        self.InX_2.setText(_translate("Main", "0"))
        self.InY_2.setText(_translate("Main", "0"))
        self.InZ_2.setText(_translate("Main", "0"))
        self.InA_2.setText(_translate("Main", "0"))
    def insertTextPID(self):
        tx = self.InX.text()
        ty = self.InY.text()
        tz = self.InZ.text()
        ta = self.InA.text()
        tg = self.comboBox.currentText()
        MoveI("PID",tx,ty,tz,ta,tg)
    def insertTextTraj(self):
        tx = self.InX_2.text()
        ty = self.InY_2.text()
        tz = self.InZ_2.text()
        ta = self.InA_2.text()
        tg = self.comboBox_2.currentText()
        MoveI("Traj",tx,ty,tz,ta,tg)
        # print(tx+', '+ty+', '+tz)
    def connectPID(self):
        Move("PID",listdata)
    def connectTraj(self):
        Move("Traj",listdata)
        
def Quit():
    end = 1

def Sethome():
    global st_Sethome
    global end
    error = 0
    data = [255,241,0,0,0,0,0,0,0,0,0]                            #"FF,F1" + x + y + z + a + g
    if st_Sethome == 1:
            msg = "Sethome Already"
            mySW.SetMsg(msg)
            return 0
    print(data)
    serialPic.write(serial.to_bytes(data))
    t = 0
    while(True):
        msg = "LUNA Loading..."
        mySW.SetMsg(msg)
        a = serialPic.readline()
        if end == 1:
            end = 0
            break
        if len(a) != 0:
            print(a)
            spic = [b for b in a][0]
            if spic == 241:
                break
            else:
                msg = "Error!"
                mySW.SetMsg(msg)
                error = 1
                break
        time.sleep(2)
        serialPic.write(serial.to_bytes(data))
        t += 1
        if t >= 2:
            break
    if error!= 1:
        st_Sethome = 1
        mySW.ui.sethomeB.setStyleSheet("background-color: rgb(100, 100, 100);\n"
            "color: rgb(255, 255, 255);")
        print("LUNA Comimg Home")
        msg = "LUNA Comimg Home"
        mySW.SetMsg(msg)

def CaptureImg(numPic):
    videoCaptureObject = cv2.VideoCapture(0)
    result = True
    path = "D:\Bachelor\Y3\Module\Commu\Pic"
    namePic = "0" + str(numPic) + ".jpg"
    while(result):
        ret,frame = videoCaptureObject.read()
        cv2.imwrite(os.path.join(path, namePic) ,frame)
        result = False
    videoCaptureObject.release()
    cv2.destroyAllWindows()

def Capture():
    global end
    data = [255,242,0,0,0,0,0,0,0,0,0]                            #"FF,F2"
    print(data)
    state = 1
    #CaptureImg(0)
    msg = "LUNA Loading..."
    mySW.SetMsg(msg)
    serialPic.write(serial.to_bytes(data))
    while(True):
        a = serialPic.readline()
        if end == 1:
            end = 0
            break
        if len(a) != 0:
            print(a)
            spic = [b for b in a][0]
            ac = [255,170,spic,0,0,0,0,0,0,0,0]
            if spic == 5:
                print("state = "+ str(state))
                msg = "state = "+ str(state)
                mySW.SetMsg(msg)
                time.sleep(1)
                #CaptureImg(state)
                serialPic.write(serial.to_bytes(ac))
                print(ac)
                break
            elif spic == state:
                print("state = "+ str(state))
                msg = "state = "+ str(state)
                mySW.SetMsg(msg)
                time.sleep(1)
                # CaptureImg(state)
                serialPic.write(serial.to_bytes(ac))
                print(ac)
                state += 1
            if spic == 99:
                print("error");
                serialPic.write(serial.to_bytes(data))
                state = 1
    print("Capture Finish")
    msg = "Capture Finish"
    mySW.SetMsg(msg)

def Catch():
    global end
    data = [255,243,0,60,0,200,0,200,0,0,255]                           #"FF,F3" + x + y + z 
    print(data)
    serialPic.write(serial.to_bytes(data))
    while(True):
        msg = "LUNA Loading..."
        mySW.SetMsg(msg)
        a = serialPic.readline()
        if end == 1:
            end = 0
            break
        if len(a) != 0:
            print(a)
            spic = [b for b in a][0]
            if spic == 243:
                break
    print("Catch!")
    msg = "Catch!"  
    mySW.SetMsg(msg)


def MoveI(m,x,y,z,a,g):
    global end
    byteL = toByte(x,y,z,a,g)
    if m == "PID":
        data = [255,244,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F4" + x + y + z
    if m == "Traj":
        data = [255,245,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F5" + x + y + z
    print(data)
    msg = "LUNA Loading..."
    mySW.SetMsg(msg)
    serialPic.write(serial.to_bytes(data))
    while(True):
        msg = "LUNA Loading..."
        mySW.SetMsg(msg)
        pic = serialPic.readline()
        if end == 1:
            end = 0
            break
        if len(pic) != 0:
            print(pic)
            spic = [b for b in pic][0]
            if m == "PID":
                if spic == 244:
                    break
            if m == "Traj":
                if spic == 245:
                    break    
    #print("Move to x="+ x +" , y="+ y + " , z="+ z + " , a="+ a + " , g="+ g)
    msg = "Move to x="+ x +" , y="+ y + " , z="+ z + " , a="+ a + " , g="+ g
    mySW.SetMsg(msg)

def Move(m,list):
    global end
    for i in list:    
        byteL = toByte(i[0],i[1],i[2],i[3],i[4])
        if m == "PID":
            data = [255,244,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F4" + x + y + z
        if m == "Traj":
            data = [255,245,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F5" + x + y + z
        print(data)
        msg = "LUNA Loading..."
        mySW.SetMsg(msg)
        serialPic.write(serial.to_bytes(data))
        while(True):
            a = serialPic.readline()
            if end == 1:
                end = 0
                break
            if len(a) != 0:
                print(a)
                spic = [b for b in a][0]
                if m == "PID":
                    if spic == 244:
                        break
                if m == "Traj":
                    if spic == 245:
                        break    
        #print("Move to x="+ x +" , y="+ y + " , z="+ z + " , a="+ a + " , g="+ g)
        msg = "Move to x="+ str(i[0]) +" , y="+ str(i[1]) + " , z="+ str(i[2]) + " , a="+ str(i[3]) + " , g="+ str(i[4])
        mySW.SetMsg(msg) 

def Reset():
    global end
    global st_Sethome
    data = [255,187,0,0,0,0,0,0,0,0,0]                            #"FF,BB" 
    print(data)
    msg = "LUNA Loading..."
    mySW.SetMsg(msg)
    serialPic.write(serial.to_bytes(data))
    while(True):
        a = serialPic.readline()
        if end == 1:
            end = 0
            break
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
    st_Sethome = 0
    mySW.ui.sethomeB.setStyleSheet("background-color: rgb(0, 85, 127);\n"
        "color: rgb(255, 255, 255);")
    t = "LUNA Reset Complete"
    mySW.SetMsg(t)

class ControlMainWindow(QMainWindow): #เพิ่มคลาสมาอีก
        def __init__(self, parent=None):
                super(ControlMainWindow, self).__init__(parent)
                self.ui =  Ui_Main() #ดึงคลาส  Ui_Form ที่เก็บรายละอียด ui มาใช้
                self.ui.setupUi(self)
        def SetMsg(self,msg):
            self.ui.textEdit.setText(msg)



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
