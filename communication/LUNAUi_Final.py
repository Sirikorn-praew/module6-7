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
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPixmap
from PyQt5.QtGui import QImage
from PyQt5.QtCore import QTimer

import cv2
import os
import imutils
import numpy as np
from camera import Camera
import mix

import serial
import time

import numpy as np
import math
from transform import four_point_transform, order_points
from arucoSub import drawBox, aruco, median, FindPath

import argparse
import imutils
import glob
from skimage import img_as_bool, io, color, morphology
from matplotlib import pyplot as plt
first_time=True

dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

cameraMatrix = np.array([[1395.3709390074625, 0.0, 984.6248356317226], [0.0, 1396.2122002126725, 534.9517311724618], [0.0, 0.0, 1.0]], np.float32) # Humanoid
dist = np.array([[0.1097213194870457, -0.1989645299789654, -0.002106454674127449, 0.004428959364733587, 0.06865838341764481]]) # Humanoid
rvec = np.array([0.0, 0.0, 0.0]) # float only
tvec = np.array([0.0, 0.0, 0.0]) # float only

serialPic = serial.Serial(port= 'COM3',timeout = 3, baudrate=115200,
                  xonxoff=0, rtscts=0,bytesize=8, parity='N', stopbits=1)
serialPic.rts = 0
serialPic.dtr = 0
serialPic.timeout = 0.1
serialPic.flush()

parameters =  cv2.aruco.DetectorParameters_create()

markerLength = 0.04
markerSeparation = 0.01

board = cv2.aruco.GridBoard_create(markersX=10, markersY=10, markerLength=markerLength, markerSeparation=markerSeparation, dictionary=dictionary)
backSub = cv2.createBackgroundSubtractorMOG2(history=100, varThreshold=16, detectShadows=False)

img_list, mask_list = [], []
imgs = []

visualize = True

st_Sethome = 0
end = 0
listdata = []



def toByte(x,y,z,a,g):
    din = [int(x),(int(y)),abs(int(z)-385),int(a)]
    bytelist = []
    for i in din:
        bytelist.append(i//256)
        bytelist.append(i%256)
    bytelist.append(int(g))
    return bytelist
# Data FF F_ xx xx yy yy zz zz aa aa gg

class Ui_Main(QWidget): #object
    def __init__(self, parent=None):
        super(Ui_Main, self).__init__(parent)
        self.sethomeB = QtWidgets.QPushButton(self)
        self.sethomeB.setGeometry(QtCore.QRect(45, 380, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.sethomeB.setFont(font)
        self.sethomeB.setMouseTracking(False)
        self.sethomeB.setStyleSheet("background-color: rgb(0, 85, 127);\n" #"border: 10px;\n""border-radius: 50;\n"
"color: rgb(255, 255, 255);")
        self.sethomeB.setCheckable(False)
        self.sethomeB.setObjectName("sethomeB")
        self.captureB = QtWidgets.QPushButton(self)
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
        self.catchB = QtWidgets.QPushButton(self)
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
        self.move_pidB = QtWidgets.QPushButton(self)
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
        self.move_trajB = QtWidgets.QPushButton(self)
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
        self.resetB = QtWidgets.QPushButton(self)
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

        self.TakePicB = QtWidgets.QPushButton("TakePic", self)
        self.TakePicB.setGeometry(QtCore.QRect(1050, 100, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.TakePicB.setFont(font)
        self.TakePicB.setMouseTracking(False)
        self.TakePicB.setStyleSheet("background-color: rgb(0, 0, 0);\n"
"color: rgb(255, 255, 255);")
        self.TakePicB.setCheckable(False)
        self.TakePicB.setObjectName("TakePicB")

        self.proPicB = QtWidgets.QPushButton("Process", self)
        self.proPicB.setGeometry(QtCore.QRect(1050, 160, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.proPicB.setFont(font)
        self.proPicB.setMouseTracking(False)
        self.proPicB.setStyleSheet("background-color: rgb(255, 182, 3);\n"
"color: rgb(255, 255, 255);")
        self.proPicB.setCheckable(False)
        self.proPicB.setObjectName("proPicB")

        self.quitB = QtWidgets.QPushButton(self)
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

        self.sent_pidB = QtWidgets.QPushButton(self)
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
        self.sent_trajB = QtWidgets.QPushButton(self)
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
        self.InX = QtWidgets.QLineEdit(self)
        self.InX.setGeometry(QtCore.QRect(520, 625, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InX.setFont(font)
        self.InX.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InX.setObjectName("InX")
        self.labelX = QtWidgets.QLabel(self)
        self.labelX.setGeometry(QtCore.QRect(480, 625, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelX.setFont(font)
        self.labelX.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelX.setObjectName("labelX")
        self.InY = QtWidgets.QLineEdit(self)
        self.InY.setGeometry(QtCore.QRect(640, 625, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InY.setFont(font)
        self.InY.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InY.setObjectName("InY")
        self.labelY = QtWidgets.QLabel(self)
        self.labelY.setGeometry(QtCore.QRect(600, 625, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelY.setFont(font)
        self.labelY.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelY.setObjectName("labelY")
        self.textEdit = QtWidgets.QTextEdit(self)
        self.textEdit.setGeometry(QtCore.QRect(550, 790, 400, 100))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.textEdit.setFont(font)
        self.textEdit.setStyleSheet("color: rgb(255, 255, 255);")
        self.textEdit.setObjectName("textEdit")
        self.InZ = QtWidgets.QLineEdit(self)
        self.InZ.setGeometry(QtCore.QRect(760, 625, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InZ.setFont(font)
        self.InZ.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InZ.setObjectName("InZ")
        self.labelZ = QtWidgets.QLabel(self)
        self.labelZ.setGeometry(QtCore.QRect(720, 625, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelZ.setFont(font)
        self.labelZ.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelZ.setObjectName("labelZ")
        self.InX_2 = QtWidgets.QLineEdit(self)
        self.InX_2.setGeometry(QtCore.QRect(520, 705, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InX_2.setFont(font)
        self.InX_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InX_2.setObjectName("InX_2")
        self.InY_2 = QtWidgets.QLineEdit(self)
        self.InY_2.setGeometry(QtCore.QRect(640, 705, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InY_2.setFont(font)
        self.InY_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InY_2.setObjectName("InY_2")
        self.labelY_2 = QtWidgets.QLabel(self)
        self.labelY_2.setGeometry(QtCore.QRect(600, 705, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelY_2.setFont(font)
        self.labelY_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelY_2.setObjectName("labelY_2")
        self.InZ_2 = QtWidgets.QLineEdit(self)
        self.InZ_2.setGeometry(QtCore.QRect(760, 705, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InZ_2.setFont(font)
        self.InZ_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InZ_2.setObjectName("InZ_2")
        self.labelX_2 = QtWidgets.QLabel(self)
        self.labelX_2.setGeometry(QtCore.QRect(480, 705, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelX_2.setFont(font)
        self.labelX_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelX_2.setObjectName("labelX_2")
        self.labelZ_2 = QtWidgets.QLabel(self)
        self.labelZ_2.setGeometry(QtCore.QRect(720, 705, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelZ_2.setFont(font)
        self.labelZ_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelZ_2.setObjectName("labelZ_2")
        self.labelA = QtWidgets.QLabel(self)
        self.labelA.setGeometry(QtCore.QRect(840, 625, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelA.setFont(font)
        self.labelA.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelA.setObjectName("labelA")
        self.InA = QtWidgets.QLineEdit(self)
        self.InA.setGeometry(QtCore.QRect(880, 625, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InA.setFont(font)
        self.InA.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InA.setObjectName("InA")
        self.InA_2 = QtWidgets.QLineEdit(self)
        self.InA_2.setGeometry(QtCore.QRect(880, 705, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.InA_2.setFont(font)
        self.InA_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.InA_2.setObjectName("InA_2")
        self.labelA_2 = QtWidgets.QLabel(self)
        self.labelA_2.setGeometry(QtCore.QRect(840, 705, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelA_2.setFont(font)
        self.labelA_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelA_2.setObjectName("labelA_2")
        self.frame = QtWidgets.QFrame(self)
        self.frame.setGeometry(QtCore.QRect(0, 0, 300, 960))
        self.frame.setStyleSheet("background-color: rgb(31, 38, 107);")
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.labelPic_2 = QtWidgets.QLabel(self.frame)
        self.labelPic_2.setGeometry(QtCore.QRect(0, 0, 300, 960))
        self.labelPic_2.setObjectName("labelPic_2")

        Lpixmap = QPixmap('L_G16.jpg')    
        self.labelPic_2.setPixmap(Lpixmap)

        self.labelPic = QtWidgets.QLabel(self)
        self.labelPic.setGeometry(QtCore.QRect(500, 90, 500, 500))
        self.labelPic.setObjectName("labelPic")

        # pixmap = QPixmap('Realy.jpg')
        self.pixmap = QPixmap('LOGO_G16.jpg')
        self.pixmap = self.pixmap.scaled(500,500)      
        self.labelPic.setPixmap(self.pixmap)

        self.comboBox = QtWidgets.QComboBox(self)
        self.comboBox.setGeometry(QtCore.QRect(1000, 625, 50, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(12)
        self.comboBox.setFont(font)
        self.comboBox.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.labelA_3 = QtWidgets.QLabel(self)
        self.labelA_3.setGeometry(QtCore.QRect(960, 625, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelA_3.setFont(font)
        self.labelA_3.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelA_3.setObjectName("labelA_3")
        self.labelA_4 = QtWidgets.QLabel(self)
        self.labelA_4.setGeometry(QtCore.QRect(960, 705, 40, 40))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.labelA_4.setFont(font)
        self.labelA_4.setStyleSheet("color: rgb(255, 255, 255);")
        self.labelA_4.setObjectName("labelA_4")
        self.comboBox_2 = QtWidgets.QComboBox(self)
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
        self.proPicB.clicked.connect(ProcessPic)
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
        self.TakePicB.raise_()
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

        self.retranslateUi(self)
        # QtCore.QMetaObject.connectSlotsByName(Main)
        # Main.setCentralWidget(self.centralwidget)

    def retranslateUi(self, Main):
        _translate = QtCore.QCoreApplication.translate
        Main.setWindowTitle(_translate("Main", "Form"))
        self.sethomeB.setText(_translate("Main", "Set home"))
        self.captureB.setText(_translate("Main", "Capture"))
        self.catchB.setText(_translate("Main", "Catch"))
        self.move_pidB.setText(_translate("Main", "Move PID"))
        self.move_trajB.setText(_translate("Main", "Move Traj"))
        self.resetB.setText(_translate("Main", "Reset"))
        # self.TakePicB.setText(_translate("Main", "TakePic"))
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
        Move("PID", listdata)
    def connectTraj(self):
        print("inconTraj")
        Move("Traj", listdata)

class Ui_Img(QWidget):
    def __init__(self, parent=None):
        super(Ui_Img, self).__init__(parent)
        self.mainB = QtWidgets.QPushButton("Main", self)
        self.mainB.setGeometry(QtCore.QRect(1050, 40, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.mainB.setFont(font)
        self.mainB.setMouseTracking(False)
        self.mainB.setStyleSheet("background-color: rgb(0, 0, 0);\n"
"color: rgb(255, 255, 255);")
        self.mainB.setCheckable(False)
        self.mainB.setObjectName("MainB")
        self.control_bt = QtWidgets.QPushButton(self)
        self.control_bt.setGeometry(QtCore.QRect(550, 800, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.control_bt.setFont(font)
        self.control_bt.setMouseTracking(False)
        self.control_bt.setStyleSheet("background-color: rgb(201, 0, 0);\n"
"color: rgb(255, 255, 255);")
        self.control_bt.setCheckable(False)
        self.control_bt.setObjectName("control_bt")

        self.capB = QtWidgets.QPushButton(self)
        self.capB.setGeometry(QtCore.QRect(550, 880, 200, 50))
        font = QtGui.QFont()
        font.setFamily("Quark")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.capB.setFont(font)
        self.capB.setMouseTracking(False)
        self.capB.setStyleSheet("background-color: rgb(255, 182, 3);\n"
"color: rgb(255, 255, 255);")
        self.capB.setCheckable(False)
        self.capB.setObjectName("capB")       
        
        self.image_label = QtWidgets.QLabel(self)
        self.image_label.setGeometry(QtCore.QRect(150, 150, 1000, 600))
        self.image_label.setObjectName("image_label")

        self.retranslateUi(self)
        # QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Main):
        _translate = QtCore.QCoreApplication.translate
        # self.image_label.setText(_translate("Main", "TextLabel"))
        self.control_bt.setText(_translate("Main", "Start"))
        self.capB.setText(_translate("Main", "Cap"))
        
def Quit():
    end = 1

def Sethome():
    global st_Sethome
    global end
    end = 0
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
            print("end")
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
        # serialPic.write(serial.to_bytes(data))
        # t += 1
        # if t >= 2:
        #     break
    if error!= 1:
        st_Sethome = 1
        mySW.ui_main.sethomeB.setStyleSheet("background-color: rgb(100, 100, 100);\n"
            "color: rgb(255, 255, 255);")
        print("LUNA Comimg Home")
        msg = "LUNA Comimg Home"
        mySW.SetMsg(msg)

def CaptureImg(numPic):
    # videoCaptureObject = cv2.VideoCapture(0) 
    videoCaptureObject = Camera()
    result = True
    path = "D:\Bachelor\Y3\Module\Commu\img"
    namePic = "0" + str(numPic) + ".jpg"
    while(result):
        frame = videoCaptureObject.read()
        cv2.imwrite(os.path.join(path, namePic) ,frame)
        result = False
    # videoCaptureObject.cap.release()
    # cv2.destroyAllWindows()

def Capture():
    global end
    end = 0
    data = [255,242,0,0,0,0,0,0,0,0,0]                            #"FF,F2"
    print(data)
    state = 1
    CaptureImg(0)
    msg = "LUNA Loading..."
    mySW.SetMsg(msg)
    serialPic.write(serial.to_bytes(data))
    while(True):
        a = serialPic.readline()
        if end == 1:
            print("end")
            end = 0
            break
        if len(a) != 0:
            print(a)
            spic = [b for b in a][0]
            ac = [255,170,spic,0,0,0,0,0,0,0,0]
            if spic == 242:
                break
            if spic == 5:
                print("state = "+ str(state))
                msg = "state = "+ str(state)
                mySW.SetMsg(msg)
                time.sleep(1)
                CaptureImg(state)
                serialPic.write(serial.to_bytes(ac))
                print(ac)
                # break
            elif spic == state:
                print("state = "+ str(state))
                msg = "state = "+ str(state)
                mySW.SetMsg(msg)
                time.sleep(1)
                CaptureImg(state)
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

def ProcessPic():
    global listdata
    aruco()
    median()
    listdata = mix.run()
    print(listdata)
    mySW.ui_main.pixmap = QPixmap('Realy.jpg')
    mySW.ui_main.pixmap = mySW.ui_main.pixmap.scaled(500,500)      
    mySW.ui_main.labelPic.setPixmap(mySW.ui_main.pixmap)
    msg = "Process Finish\n" + str(listdata)
    mySW.SetMsg(msg)


def Catch():
    global end, listdata
    byteL = toByte(listdata[0][0],(int(listdata[0][1])-15),(int(listdata[0][2])+100),listdata[0][3],255)
    data = [255,243,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]                           #"FF,F3" + x + y + z 
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
    print("input is comimggg")
    print(list)
    for i in list:    
        byteL = toByte((int(i[0])-10),(int(i[1])-15),(int(i[2])+100),i[3],255)
        if m == "PID":
            data = [255,244,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F4" + x + y + z
        if m == "Traj":
            data = [255,245,byteL[0],byteL[1],byteL[2],byteL[3],byteL[4],byteL[5],byteL[6],byteL[7],byteL[8]]    #"FF,F5" + x + y + z
        print(data)
        x = byteL[0] + byteL[1]
        y = byteL[2] + byteL[3]
        z = byteL[4] + byteL[5]
        # msg = "LUNA Loading..."
        msg = "Move to x="+ str(x) +" , y="+ str(y) + " , z="+ str(z) + " , a="+ str(i[3]) + " , g="+ str(255)
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
        msg = "Move to x="+ str(x) +" , y="+ str(y) + " , z="+ str(z) + " , a="+ str(i[3]) + " , g="+ str(255)
        mySW.SetMsg(msg) 

def Reset():
    global end
    global st_Sethome
    end = 0
    data = [255,187,0,0,0,0,0,0,0,0,0]                            #"FF,BB" 
    print(data)
    msg = "LUNA Loading..."
    mySW.SetMsg(msg)
    serialPic.write(serial.to_bytes(data))
    while(True):
        a = serialPic.readline()
        if end == 1:
            print("end")
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
    mySW.ui_main.sethomeB.setStyleSheet("background-color: rgb(0, 85, 127);\n"
        "color: rgb(255, 255, 255);")
    t = "LUNA Reset Complete"
    mySW.SetMsg(t)

class Main(QMainWindow): #เพิ่มคลาสมาอีก
        def __init__(self, parent=None):
                super(Main, self).__init__(parent)
                self.setGeometry(500, 50, 400, 450)
                self.resize(1280, 960)
                self.setStyleSheet("background-color: rgb(48, 48, 48);")
                self.startUiMain()

        def startUiImg(self):
            self.ui_img = Ui_Img(self)
            self.setWindowTitle("Image")
            self.setCentralWidget(self.ui_img)
            # self.ui_img.setupUi(self)
            self.ui_img.mainB.clicked.connect(self.startUiMain)
            # create a timer
            self.timer = QTimer()
            # set timer timeout callback function
            self.timer.timeout.connect(self.viewCam)
            # set control_bt callback clicked  function
            self.ui_img.control_bt.clicked.connect(self.controlTimer)
            self.ui_img.capB.clicked.connect(self.Cap)
            self.show()

        def startUiMain(self):
            self.ui_main =  Ui_Main(self)
            self.setWindowTitle("Main")
            self.setCentralWidget(self.ui_main)
            # self.ui_main.setupUi(self)
            self.ui_main.TakePicB.clicked.connect(self.startUiImg)
            self.show()

        def SetMsg(self,t):
            self.ui_main.textEdit.setText(t)
        
        # view camera
        def viewCam(self):
            # read image in BGR format
            ret, image = self.cap.read()
            # convert image to RGB format
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            rect_mask = cv2.rectangle(np.ones_like(image)*255, (430, 220), (630, 420), (0,0,0), -1)
            alpha = 0.5
            cv2.addWeighted(rect_mask, alpha, image, 1-alpha,0,image)
            # get image infos
            height, width, channel = image.shape
            step = channel * width
            # create QImage from image
            qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)
            # show image in img_label
            self.ui_img.image_label.setPixmap(QPixmap.fromImage(qImg))
        
        def controlTimer(self):
            cap = Camera()
            image = cap.read()
            # if timer is stopped
            if not self.timer.isActive():
                # create video capture
                self.cap = cap.cap
                # start timer
                self.timer.start(20)
                # update control_bt text
                self.ui_img.control_bt.setText("Stop")
            # if timer is started
            else:
                # stop timer
                self.timer.stop()
                # release video capture
                self.cap.release()
                # update control_bt text
                self.ui_img.control_bt.setText("Start")
        def Cap(self):
            ret, image = self.cap.read()
            resized = imutils.resize(image[220:420, 430:630], height=200)
            cv2.imwrite("card.jpg", resized)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    mySW = Main() 
    sys.exit(app.exec_())

