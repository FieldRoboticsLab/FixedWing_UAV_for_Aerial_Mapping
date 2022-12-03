#                    #
#   Ground Station   #
#                    #

# 2019 ---> ?
#Author: Umut Dumandağ
#Mail: umutdumandag61@gmail.com
#Github page: https://github.com/UmutDumandag

from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import *
import sys
import QT_designer.icons_rc
import QT_designer.icons_1

from QT_designer.qroundprogressbar import QRoundProgressBar
from pyqtlet import L, MapWidget
from qfi import qfi_ADI, qfi_ALT, qfi_SI, qfi_HSI, qfi_VSI, qfi_TC          
from PyQt5.QtCore import pyqtSlot
import time
from threading import Thread
import serial
import math
import glob
import pyqtgraph as pg
import geopy.distance
import random

#import redis
from playsound import playsound
from src import SurveyGrid

from dronekit import connect, Command

connection_string = "/dev/ttyUSB0"

JS = """
                    // save these original methods before they are overwritten
            var proto_initIcon = L.Marker.prototype._initIcon;
            var proto_setPos = L.Marker.prototype._setPos;

            var oldIE = (L.DomUtil.TRANSFORM === 'msTransform');

            L.Marker.addInitHook(function () {
                var iconOptions = this.options.icon && this.options.icon.options;
                var iconAnchor = iconOptions && this.options.icon.options.iconAnchor;
                if (iconAnchor) {
                    iconAnchor = (iconAnchor[0] + 'px ' + iconAnchor[1] + 'px');
                }
                this.options.rotationOrigin = this.options.rotationOrigin || iconAnchor || 'center bottom' ;
                this.options.rotationAngle = this.options.rotationAngle || 0;

                // Ensure marker keeps rotated during dragging
                this.on('drag', function(e) { e.target._applyRotation(); });
            });

            L.Marker.include({                                                                                             
                _initIcon: function() {
                    proto_initIcon.call(this);
                },

                _setPos: function (pos) {
                    proto_setPos.call(this, pos);
                    this._applyRotation();
                },

                _applyRotation: function () {
                    if(this.options.rotationAngle) {
                        this._icon.style[L.DomUtil.TRANSFORM+'Origin'] = this.options.rotationOrigin;

                        if(oldIE) {
                            // for IE 9, use the 2D rotation
                            this._icon.style[L.DomUtil.TRANSFORM] = 'rotate(' + this.options.rotationAngle + 'deg)';
                        } else {
                            // for modern browsers, prefer the 3D accelerated version
                            this._icon.style[L.DomUtil.TRANSFORM] += ' rotateZ(' + this.options.rotationAngle + 'deg)';
                        }
                    }
                },

                setRotationAngle: function(angle) {
                    this.options.rotationAngle = angle;
                    this.update();
                    return this;
                },

                setRotationOrigin: function(origin) {
                    this.options.rotationOrigin = origin;
                    this.update();
                    return this;
                }
            });
"""

class EmbTerminal(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(EmbTerminal, self).__init__(parent)
        self.process = QtCore.QProcess(self)
        self.terminal = QtWidgets.QWidget(self)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.terminal)
        # Works also with urxvt:
        self.process.start('urxvt',['-embed', str(int(self.winId()))])
        self.setFixedSize(490, 431)

    def stop(self):
        self.process.kill()

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        #self.r = redis.Redis()
        self.window_()
        self.toolbar_()
        self.stackedWidget_()
        self.map_()
        self.tab_()
        self.frame_()
        self.gridlayout_()
        self.scroll_area_()
        self.label_()
        self.lineedit_()
        self.buttonbox_()
        self.lcdnumber_()
        self.pushbutton_()
        self.textbrowser_()
        self.widgets_()
        self.stackedWidget.addWidget(self.page_1)
        self.stackedWidget.addWidget(self.page_2)
        self.stackedWidget.addWidget(self.page_3)
        self.serial_ports()
        self.createPlan_()
        self.createPlan_A()
        self.createSurveyGrid_()
        self.loadMission_()
        self.editMission_()

        self.connect_()
        self.arr = [False] * 32
        self.arr_3 = []
        self.number = 0
        self.number_follow = 0
        self.follow_flag = True
        self.y = [0]
        self.y_2 = [0]
        self.y_3 = [0]                                                                                   
        self.y_4 = [0]
        self.y_5 = [0]

        self.x = [0]
        self.x_2 = [0]
        self.x_3 = [0]
        self.x_4 = [0]
        self.x_5 = [0]
        self.graphics_()
        #self.colors = ["red", "yellow", "blue", "orange", "green", "purple", "brown", "pink", "grey", "#FF0000"]
        self.colors = ["#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)]) for i in range(50)]
        self.vehicle = None

    def window_(self):
        self.setWindowTitle("Ground Station")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/newPrefix/pp.jfif"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.setWindowIcon(icon)

    def download_mission(self):
        """
        Downloads the current mission and returns it in a list.
        It is used in save_mission() to get the file information to save.
        """
        print(" Download mission from vehicle")
        missionlist=[]
        cmds = self.vehicle.commands
        cmds.download()
        for cmd in cmds:
            missionlist.append(cmd)
        return missionlist

    def save_mission(self, aFileName):
        """
        Save a mission in the Waypoint file format 
        (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
        """
        print("\nSave mission from Vehicle to file: %s" % aFileName)    
        #Download mission from vehicle
        missionlist = self.download_mission()
        #Add file-format information
        output='QGC WPL 110\n'
        #Add home location as 0th waypoint
        #home = vehicle.home_location
        #output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
        #Add commands
        for cmd in missionlist:
            commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
            output+=commandline
        with open(aFileName, 'w') as file_:
            print(" Write mission to file")
            file_.write(output)
            
        
    def printfile(self, aFileName):
        """
        Print a mission file to demonstrate "round trip"
        """
        print("\nMission file: %s" % aFileName)
        with open(aFileName) as f:
            self.plan_textbrowser_load.append("--------------------")
            for line in f:
                self.plan_textbrowser_load.append(line.strip())
                #print(' %s' % line.strip())        
            self.plan_textbrowser_load.append("--------------------")

    def upload_mission(self, aFileName):
        """
        Upload a mission from a file. 
        """
        #Read mission from file
        missionlist = self.readmission(aFileName)
        
        print("\nUpload mission from a file: %s" % aFileName)
        #Clear existing mission from vehicle
        print(' Clear mission')
        cmds = self.vehicle.commands
        cmds.clear()
        #Add new mission to vehicle
        for command in missionlist:
            cmds.add(command)
        print(' Upload mission')
        self.vehicle.commands.upload()

    def readmission(self, aFileName):
        """
        Load a mission from a file into a list. The mission definition is in the Waypoint file
        format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

        This function is used by upload_mission().
        """
        print("\nReading mission from file: %s" % aFileName)
        cmds = self.vehicle.commands
        missionlist=[]
        with open(aFileName) as f:
            for i, line in enumerate(f):
                if i==0:
                    if not line.startswith('QGC WPL 110'):
                        raise Exception('File is not supported WP version')
                else:
                    linearray=line.split('\t')
                    ln_index=int(linearray[0])
                    ln_currentwp=int(linearray[1])
                    ln_frame=int(linearray[2])
                    ln_command=int(linearray[3])
                    ln_param1=float(linearray[4])
                    ln_param2=float(linearray[5])
                    ln_param3=float(linearray[6])
                    ln_param4=float(linearray[7])
                    ln_param5=float(linearray[8])
                    ln_param6=float(linearray[9])
                    ln_param7=float(linearray[10])
                    ln_autocontinue=int(linearray[11].strip())
                    cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                    missionlist.append(cmd)
        return missionlist

    def toolbar_(self):
        self.toolBar = QtWidgets.QToolBar(self)
        self.toolBar.setMovable(False)
        self.toolBar.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolBar.setObjectName("toolBar")
        self.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)

        self.home_actionmain = QtWidgets.QAction(self)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/newPrefix/unnamed.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.home_actionmain.setIcon(icon)
        self.home_actionmain.setObjectName("actionmain")
        self.toolBar.addAction(self.home_actionmain)
        self.home_actionmain.setText(".         HOME         .")

        self.spacer = QtWidgets.QWidget()
        self.spacer.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.toolBar.addWidget(self.spacer)

        self.coordinates_actionmain = QtWidgets.QAction(self)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/newPrefix/images.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.coordinates_actionmain.setIcon(icon)
        self.coordinates_actionmain.setObjectName("actionmain_2")
        self.toolBar.addAction(self.coordinates_actionmain)
        self.coordinates_actionmain.setText("COORDİNATES")

        self.spacer_2 = QtWidgets.QWidget()
        self.spacer_2.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.toolBar.addWidget(self.spacer_2)

        self.port_actionmain = QtWidgets.QAction(self)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/newPrefix/computer-icons.jpg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.port_actionmain.setIcon(icon)
        self.port_actionmain.setObjectName("actionmain_3")
        self.toolBar.addAction(self.port_actionmain)

    def stackedWidget_(self):
        self.stackedWidget = QtWidgets.QStackedWidget(self)
        self.stackedWidget.setGeometry(QtCore.QRect(0, 55, 1920, 1080))
        self.stackedWidget.setObjectName("stackedWidget")

        self.page_1 = QtWidgets.QWidget()
        self.page_1.setObjectName("page_1")

        self.page_2 = QtWidgets.QWidget()
        self.page_2.setObjectName("page_2")

        self.page_3 = QtWidgets.QWidget()
        self.page_3.setObjectName("page_3")


    def map_(self):
        self.map_frame = QtWidgets.QFrame(self.page_1)
        self.map_frame.setGeometry(QtCore.QRect(0, 7, 1468, 445))
        self.map_layoutwidget = QtWidgets.QWidget(self.map_frame)
        self.map_layoutwidget.setGeometry(QtCore.QRect(0, 0, 1478, 474))
        self.map_layoutwidget.setObjectName("map_layoutwidget")
        self.map_layout = QtWidgets.QGridLayout(self.map_layoutwidget)
        self.map_layout.setContentsMargins(0, 0, 0, 0)
        self.mapWidget = MapWidget()
        self.map_layout.addWidget(self.mapWidget, 0, 0)

        self.map = L.map(self.mapWidget)
        #self.map.setMaxZoom(20)
        self.map.setView([40.94713851148793, 29.399159585576985], 15) # -35.36164262672604, 149.16122176277864
        self.map.runJavaScript(JS)
        #http://mt0.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}&s=Ga
        L.tileLayer("https://api.mapbox.com/styles/v1/mapbox/satellite-streets-v11/tiles/512/{z}/{x}/{y}?access_token=pk.eyJ1IjoidW11dGR1bWFuZGFnIiwiYSI6ImNrZWNxdW9iNjBrbmkzM210cWoxcG4zYWEifQ.iWCJCE3S8o8Oo_oLirGoUQ", options="{maxNativeZoom : 20, maxZoom: 22}").addTo(self.map) 
        #"https://api.mapbox.com/styles/v1/mapbox/satellite-streets-v11/tiles/512/{z}/{x}/{y}?access_token=pk.eyJ1IjoidW11dGR1bWFuZGFnIiwiYSI6ImNrZWNxdW9iNjBrbmkzM210cWoxcG4zYWEifQ.iWCJCE3S8o8Oo_oLirGoUQ"
        # http://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png 4uMaps/{z}/{x}/{y}.png 
        # http://localhost:8080/styles/osm-bright/{z}/{x}/{y}.png  
        # http://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}
        self.drawControl = L.control.draw()
        self.map.addControl(self.drawControl)
        self.map.clicked.connect(lambda x: self.fonk(x))

    def fonk(self, x):
        self.arr_3.append(x)
        self.all_spe = self.arr_3[self.number]
        #print("{}. waypoint : {}".format(self.number, self.all_spe['latlng']))
        #self.console_textbrowser.append("You clicked this location <----> {}".format(self.all_spe['latlng']))
        self.map_lat_lng_label.setText("{}".format(self.all_spe['latlng']))
        self.lineEdit_createPlan.setText("{}".format(self.all_spe['latlng']))
        self.lineEdit_SurveyGrid_lat_lng.setText("{}".format(self.all_spe['latlng']))
        self.lon_edit_name.setText(str(self.all_spe['latlng']['lat']))
        self.lng_edit_name.setText(str(self.all_spe['latlng']['lng']))
        self.number += 1

    def tab_(self):
        stylesheet = """ 
            QTabBar::tab:selected {background: yellow;}
            QTabWidget>QWidget>QWidget{background: gray;}
        """

        self.tabWidget = QtWidgets.QTabWidget(self.page_1)
        self.tabWidget.setStyleSheet(stylesheet)
        self.tabWidget.setGeometry(QtCore.QRect(-5, 455, 1000, 520)) #490
        self.tabWidget.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.tabWidget.setMovable(True)
        self.tabWidget.setObjectName("tabWidget")

        self.missions_tab = QtWidgets.QWidget()
        self.missions_tab.setObjectName("tab")
        self.missions_tab.setStyleSheet('background-color: rgb(0,0,0)')
        self.tabWidget.addTab(self.missions_tab, "")

        self.widgets_tab = QtWidgets.QWidget()
        self.widgets_tab.setObjectName("tab_2")
        self.tabWidget.addTab(self.widgets_tab, "")
        self.widgets_tab.setStyleSheet('background-color: rgb(0, 0, 0)')

        self.graphics_tab = QtWidgets.QWidget()
        self.graphics_tab.setObjectName("tab_3")
        self.tabWidget.addTab(self.graphics_tab, "")
        self.graphics_tab.setStyleSheet('background-color: rgb(0, 0, 0)')

        self.createPlan_tab = QtWidgets.QWidget()
        self.createPlan_tab.setObjectName("tab_4")
        self.tabWidget.addTab(self.createPlan_tab, "")
        self.createPlan_tab.setStyleSheet('background-color: rgb(0, 0, 0)')

        self.createPlanA_tab = QtWidgets.QWidget()
        self.createPlanA_tab.setObjectName("tab_5")
        self.tabWidget.addTab(self.createPlanA_tab, "")
        self.createPlanA_tab.setStyleSheet('background-color: rgb(0, 0, 0)')

        self.createSurveyGrid_tab = QtWidgets.QWidget()
        self.createSurveyGrid_tab.setObjectName("tab_5")
        self.tabWidget.addTab(self.createSurveyGrid_tab, "")
        self.createSurveyGrid_tab.setStyleSheet('background-color: rgb(0, 0, 0)')

        self.loadMission_tab = QtWidgets.QWidget()
        self.loadMission_tab.setObjectName("tab_5")
        self.tabWidget.addTab(self.loadMission_tab, "")
        self.loadMission_tab.setStyleSheet('background-color: rgb(0, 0, 255)')


        self.editMission_tab = QtWidgets.QWidget()
        self.editMission_tab.setObjectName("tab_6")
        self.tabWidget.addTab(self.editMission_tab, "")
        self.editMission_tab.setStyleSheet('background-color: rgb(255, 0, 0)')

        self.tabWidget.setTabText(self.tabWidget.indexOf(self.missions_tab), ("Missions"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.widgets_tab), ("Widgets"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.graphics_tab), ("Graphics"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.createPlan_tab), ("Create Plan PX4"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.createPlanA_tab), ("Create Plan Ardupilot"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.createSurveyGrid_tab), ("Survey Grid"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.loadMission_tab), ("Upload Mission"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.editMission_tab), ("Edit Mission"))
    
    def frame_(self):
        self.main_frame_right = QtWidgets.QFrame(self.page_1)
        self.main_frame_right.setGeometry(QtCore.QRect(1470, 5, 400, 965))
        self.main_frame_right.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.main_frame_right.setFrameShadow(QtWidgets.QFrame.Raised)
        self.main_frame_right.setObjectName("frame")
        self.main_frame_right.setStyleSheet('background-color: rgb(0, 0, 0)')

        self.console_frame = QtWidgets.QFrame(self.page_1)
        self.console_frame.setGeometry(QtCore.QRect(990, 483, 480, 478))
        self.console_frame.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.console_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.console_frame.setStyleSheet('background-color: rgb(0, 0, 0)')

        self.mission1_frame = QtWidgets.QFrame(self.missions_tab)
        self.mission1_frame.setGeometry(QtCore.QRect(10, 20, 431, 430))
        self.mission1_frame.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.mission1_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.mission1_frame.setObjectName("frame_6")
        self.mission1_frame.setStyleSheet('background-color: rgb(99, 184, 255)')

        self.gridLayoutWidget_4 = QtWidgets.QWidget(self.mission1_frame)
        self.gridLayoutWidget_4.setGeometry(QtCore.QRect(20, 20, 395, 81))
        self.gridLayoutWidget_4.setObjectName("gridLayoutWidget_4")

        self.gridLayout_4 = QtWidgets.QGridLayout(self.gridLayoutWidget_4)
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_4.setObjectName("gridLayout_4")

        self.mission1_status_frame = QtWidgets.QFrame(self.gridLayoutWidget_4)
        self.mission1_status_frame.setStyleSheet("background-color: rgb(255, 0, 0);")
        self.mission1_status_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.mission1_status_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.mission1_status_frame.setObjectName("frame_7")
        self.gridLayout_4.addWidget(self.mission1_status_frame, 0, 3, 1, 1)

        self.mission1_picture_frame = QtWidgets.QFrame(self.mission1_frame)
        self.mission1_picture_frame.setGeometry(QtCore.QRect(5, 110, 421, 250))
        self.mission1_picture_frame.setStyleSheet("image: url(:/newPrefix/uav.png);")
        self.mission1_picture_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.mission1_picture_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.mission1_picture_frame.setObjectName("frame_8")

        self.mission2_frame = QtWidgets.QFrame(self.missions_tab)
        self.mission2_frame.setGeometry(QtCore.QRect(540, 20, 431, 430))
        self.mission2_frame.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.mission2_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.mission2_frame.setObjectName("frame_9")
        self.mission2_frame.setStyleSheet('background-color: rgb(99, 184, 255)')

        self.gridLayoutWidget_6 = QtWidgets.QWidget(self.mission2_frame)
        self.gridLayoutWidget_6.setGeometry(QtCore.QRect(20, 20, 395, 81))
        self.gridLayoutWidget_6.setObjectName("gridLayoutWidget_6")

        self.gridLayout_6 = QtWidgets.QGridLayout(self.gridLayoutWidget_6)
        self.gridLayout_6.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_6.setObjectName("gridLayout_6")

        self.mission2_status_frame = QtWidgets.QFrame(self.gridLayoutWidget_6)
        self.mission2_status_frame.setStyleSheet("background-color: rgb(255, 0, 0);")
        self.mission2_status_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.mission2_status_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.mission2_status_frame.setObjectName("frame_12")
        self.gridLayout_6.addWidget(self.mission2_status_frame, 0, 3, 1, 1)

        self.mission2_picture_frame = QtWidgets.QFrame(self.mission2_frame)
        self.mission2_picture_frame.setGeometry(QtCore.QRect(5, 110, 421, 250))
        self.mission2_picture_frame.setStyleSheet("image: url(:/newPrefix/gazebo.png);")
        self.mission2_picture_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.mission2_picture_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.mission2_picture_frame.setObjectName("frame_10")

        self.coordinates_frame = QtWidgets.QFrame(self.page_2)
        self.coordinates_frame.setGeometry(QtCore.QRect(10, 0, 1910, 331))
        self.coordinates_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.coordinates_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.coordinates_frame.setFrameStyle(QFrame.NoFrame)
        self.coordinates_frame.setObjectName("frame")

    def gridlayout_(self):
        self.gridLayoutWidget_1 = QtWidgets.QWidget(self.main_frame_right)
        self.gridLayoutWidget_1.setGeometry(QtCore.QRect(40, 90, 326, 281))
        self.gridLayoutWidget_1.setObjectName("gridLayoutWidget_1")

        self.gridLayout_1 = QtWidgets.QGridLayout(self.gridLayoutWidget_1)
        self.gridLayout_1.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_1.setObjectName("gridLayout")

        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.main_frame_right)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(40, 430, 326, 281))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")

        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")

        self.gridLayoutWidget_3 = QtWidgets.QWidget(self.main_frame_right)
        self.gridLayoutWidget_3.setGeometry(QtCore.QRect(40, 780, 476, 121))#40, 780, 301, 121
        self.gridLayoutWidget_3.setObjectName("gridLayoutWidget_3")

        self.gridLayout_3 = QtWidgets.QGridLayout(self.gridLayoutWidget_3)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_3.setObjectName("gridLayout")


        self.gridLayoutWidget_5 = QtWidgets.QWidget(self.mission1_frame)
        self.gridLayoutWidget_5.setGeometry(QtCore.QRect(30, 360, 381, 51))
        self.gridLayoutWidget_5.setObjectName("gridLayoutWidget_5")

        self.gridLayout_5 = QtWidgets.QGridLayout(self.gridLayoutWidget_5)
        self.gridLayout_5.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_5.setObjectName("gridLayout_5")


        self.gridLayoutWidget_7 = QtWidgets.QWidget(self.mission2_frame)
        self.gridLayoutWidget_7.setGeometry(QtCore.QRect(30, 360, 381, 51))
        self.gridLayoutWidget_7.setObjectName("gridLayoutWidget_7")

        self.gridLayout_7 = QtWidgets.QGridLayout(self.gridLayoutWidget_7)
        self.gridLayout_7.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_7.setObjectName("gridLayout_7")

        self.gridLayoutWidget_8 = QtWidgets.QWidget(self.widgets_tab)
        self.gridLayoutWidget_8.setGeometry(QtCore.QRect(0, 2, 240, 240))
        self.gridLayoutWidget_8.setObjectName("gridLayoutWidget_8")

        self.tab2_layout = QtWidgets.QGridLayout(self.gridLayoutWidget_8)

        self.gridLayoutWidget_9 = QtWidgets.QWidget(self.widgets_tab)
        self.gridLayoutWidget_9.setGeometry(QtCore.QRect(250, 2, 240, 240))
        self.gridLayoutWidget_9.setObjectName("gridLayoutWidget_9")

        self.tab2_layout2 = QtWidgets.QGridLayout(self.gridLayoutWidget_9)

        self.gridLayoutWidget_10 = QtWidgets.QWidget(self.widgets_tab)
        self.gridLayoutWidget_10.setGeometry(QtCore.QRect(500, 2, 240, 240))
        self.gridLayoutWidget_10.setObjectName("gridLayoutWidget_10")

        self.tab2_layout3 = QtWidgets.QGridLayout(self.gridLayoutWidget_10)

        self.gridLayoutWidget_11 = QtWidgets.QWidget(self.widgets_tab)
        self.gridLayoutWidget_11.setGeometry(QtCore.QRect(0, 240, 240, 240))
        self.gridLayoutWidget_11.setObjectName("gridLayoutWidget_11")

        self.tab2_layout4 = QtWidgets.QGridLayout(self.gridLayoutWidget_11)

        self.gridLayoutWidget_12 = QtWidgets.QWidget(self.widgets_tab)
        self.gridLayoutWidget_12.setGeometry(QtCore.QRect(250, 240, 240, 240))
        self.gridLayoutWidget_12.setObjectName("gridLayoutWidget_12")

        self.tab2_layout5 = QtWidgets.QGridLayout(self.gridLayoutWidget_12)

        self.gridLayoutWidget_13 = QtWidgets.QWidget(self.widgets_tab)
        self.gridLayoutWidget_13.setGeometry(QtCore.QRect(500, 240, 240, 240))
        self.gridLayoutWidget_13.setObjectName("gridLayoutWidget_13")

        self.tab2_layout6 = QtWidgets.QGridLayout(self.gridLayoutWidget_13)

        self.gridLayoutWidget_14 = QtWidgets.QWidget(self.widgets_tab)
        self.gridLayoutWidget_14.setGeometry(QtCore.QRect(800, 0, 200, 200))
        self.gridLayoutWidget_14.setObjectName("gridLayoutWidget_14")

        self.tab2_layout7 = QtWidgets.QGridLayout(self.gridLayoutWidget_14)

        self.gridLayoutWidget_15 = QtWidgets.QWidget(self.widgets_tab)
        self.gridLayoutWidget_15.setGeometry(QtCore.QRect(800, 235, 200, 200))
        self.gridLayoutWidget_15.setObjectName("gridLayotWidget_15")

        self.tab2_layout8 = QtWidgets.QGridLayout(self.gridLayoutWidget_15)

        self.gridLayoutWidget_16 = QtWidgets.QWidget(self.coordinates_frame)
        self.gridLayoutWidget_16.setGeometry(QtCore.QRect(0, 80, 1700, 31))
        self.gridLayoutWidget_16.setObjectName("gridLayoutWidget_16")

        self.gridLayout_9 = QtWidgets.QGridLayout(self.gridLayoutWidget_16)
        self.gridLayout_9.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_9.setObjectName("gridLayout_9")

        self.gridLayoutWidget_17 = QtWidgets.QWidget(self.coordinates_frame)
        self.gridLayoutWidget_17.setGeometry(QtCore.QRect(0, 206, 1700, 31))
        self.gridLayoutWidget_17.setObjectName("gridLayoutWidget_17")

        self.gridLayout_10 = QtWidgets.QGridLayout(self.gridLayoutWidget_17)
        self.gridLayout_10.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_10.setObjectName("gridLayout_10")


    def scroll_area_(self):
        self.graphic_scrollArea = QScrollArea(self.graphics_tab)
        self.graphic_scrollArea.setObjectName(u"scrollArea")
        self.graphic_scrollArea.setGeometry(QtCore.QRect(5, 20, 970, 431))
        self.graphic_scrollArea.setWidgetResizable(False)
        self.graphic_scrollArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
        self.scrollAreaWidgetContents = QWidget()
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(5, 0, 940, 2170))

        self.gridLayoutWidget_18 = QWidget(self.scrollAreaWidgetContents)
        self.gridLayoutWidget_18.setObjectName("gridLayoutWidget_18")
        self.gridLayoutWidget_18.setGeometry(QtCore.QRect(5, 0, 920, 2150))
        self.gridLayout_11 = QGridLayout(self.gridLayoutWidget_18)
        self.gridLayout_11.setObjectName("gridLayout_11")


    def graphics_(self):
        pen = pg.mkPen(color=(255, 0, 0))
        self.graphWidget = pg.PlotWidget()
        self.gridLayout_11.addWidget(self.graphWidget, 1, 0)
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setTitle("SECOND-ALTİTUDE")
        self.graphWidget.setLabel('left', "<span style=\"color:red;font-size:20px\">Altitude (m)</span>")
        self.graphWidget.setLabel('bottom', "<span style=\"color:red;font-size:20px\">Second (sn)</span>")

        self.x = [i + 1 for i in range(len(self.y))]

        self.data_line = self.graphWidget.plot(self.x,
                              self.y,
                              pen=pen,
                              symbol='o',
                              fillLevel=0,
                              brush=(50,50,200,200))

        self.graphWidget_2 = pg.PlotWidget()
        self.gridLayout_11.addWidget(self.graphWidget_2, 2,0)
        self.graphWidget_2.showGrid(x=True, y=True)
        self.graphWidget_2.setTitle("SECOND-SPEED X")
        self.graphWidget_2.setLabel('left', "<span style=\"color:red;font-size:20px\">Speed X (m/sn)</span>")
        self.graphWidget_2.setLabel('bottom', "<span style=\"color:red;font-size:20px\">Second (sn)</span>")

        self.x_2 = [i + 1 for i in range(len(self.y_2))]

        self.data_line_2 = self.graphWidget_2.plot(self.x_2,
                              self.y_2,
                              pen=pen,
                              symbol='o',
                              fillLevel=0,
                              brush=(50, 50, 200, 200))



        self.graphWidget_3 = pg.PlotWidget()
        self.gridLayout_11.addWidget(self.graphWidget_3, 3,0)
        self.graphWidget_3.showGrid(x=True, y=True)
        self.graphWidget_3.setTitle("SECOND-SPEED Y")
        self.graphWidget_3.setLabel('left', "<span style=\"color:red;font-size:20px\">Speed Y (m/sn)</span>")
        self.graphWidget_3.setLabel('bottom', "<span style=\"color:red;font-size:20px\">Second (sn)</span>")

        self.x_3 = [i + 1 for i in range(len(self.y_3))]

        self.data_line_3 = self.graphWidget_3.plot(self.x_3,
                              self.y_3,
                              pen=pen,
                              symbol='o',
                              fillLevel=0,
                              brush=(50, 50, 200, 200))


        self.data_line_4 = self.graphWidget_4 = pg.PlotWidget()
        self.gridLayout_11.addWidget(self.graphWidget_4, 4,0)
        self.graphWidget_4.showGrid(x=True, y=True)
        self.graphWidget_4.setTitle("SECOND-SPEED Z")
        self.graphWidget_4.setLabel('left', "<span style=\"color:red;font-size:20px\">Speed Z (m/sn)</span>")
        self.graphWidget_4.setLabel('bottom', "<span style=\"color:red;font-size:20px\">Second (sn)</span>")

        self.x_4 = [i + 1 for i in range(len(self.y_4))]

        self.data_line_4 = self.graphWidget_4.plot(self.x_4,
                              self.y_4,
                              pen=pen,
                              symbol='o',
                              fillLevel=0,
                              brush=(50, 50, 200, 200))

        self.graphWidget_5 = pg.PlotWidget()
        self.gridLayout_11.addWidget(self.graphWidget_5, 5,0)
        self.graphWidget_5.showGrid(x=True, y=True)
        self.graphWidget_5.setTitle("SECOND-BATTERY CURRENT")
        self.graphWidget_5.setLabel('left', "<span style=\"color:red;font-size:20px\">Battery Current (V)</span>")
        self.graphWidget_5.setLabel('bottom', "<span style=\"color:red;font-size:20px\">Second (sn)</span>")

        self.x_5 = [i + 1 for i in range(len(self.y_5))]

        self.data_line_5 = self.graphWidget_5.plot(self.x_5,
                              self.y_5,
                              pen=pen,
                              symbol='o',
                              fillLevel=0,
                              brush=(50, 50, 200, 200))

        self.graphic_scrollArea.setWidget(self.scrollAreaWidgetContents)

    def lineedit_(self):
        self.lineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget_16)
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit.setText("[[37.072978,37.264369],[37.072978,37.2736892700],[37.074698,37.275651],[37.075713,37.273601],[37.075300,37.264245],[37.074129,37.262759]]")
        self.gridLayout_9.addWidget(self.lineEdit, 0, 1, 1, 1)

        self.lineEdit_2 = QtWidgets.QLineEdit(self.gridLayoutWidget_17)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.lineEdit_2.setText("[[37.072978,37.264369],[37.072978,37.2736892700],[37.074698,37.275651],[37.075713,37.273601],[37.075300,37.264245],[37.074129,37.262759]]")
        self.gridLayout_10.addWidget(self.lineEdit_2, 0, 1, 1, 1)
        

        self.lineEdit_3 = QtWidgets.QLineEdit(self.coordinates_frame)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.lineEdit_3.setReadOnly(True)
        self.lineEdit_3.setGeometry(QtCore.QRect(490, 300, 1000, 30))
        self.lineEdit_3.setText("[[37.072978,37.264369],[37.072978,37.2736892700],[37.074698,37.275651],[37.075713,37.273601],[37.075300,37.264245],[37.074129,37.262759]]")

    def buttonbox_(self):
        self.buttonBox = QtWidgets.QDialogButtonBox(self.coordinates_frame)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel | QtWidgets.QDialogButtonBox.Save)
        self.buttonBox.setObjectName("buttonBox")
        self.buttonBox.setGeometry(QtCore.QRect(1360, 80, 541, 31))

        self.buttonBox_2 = QtWidgets.QDialogButtonBox(self.coordinates_frame)
        self.buttonBox_2.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel | QtWidgets.QDialogButtonBox.Save)
        self.buttonBox_2.setObjectName("buttonBox_2")
        self.buttonBox_2.setGeometry(QtCore.QRect(1360, 206, 541, 31))

    def createSurveyGrid_(self):

        font = QtGui.QFont()
        font.setBold(True)
        font.setPointSize(9)
        font.setWeight(20)

        self.createSurveyGrid_tab.setStyleSheet('background-color : rgb(15, 253, 70)')
        self.lineEdit_SurveyGrid_lat_lng = QtWidgets.QLineEdit(self.createSurveyGrid_tab)
        self.lineEdit_SurveyGrid_lat_lng.setObjectName("lineEdit_SurveyGrid_lat_lng")
        self.lineEdit_SurveyGrid_lat_lng.setGeometry(QtCore.QRect(50, 100, 600, 30))
        self.lineEdit_SurveyGrid_lat_lng.setStyleSheet('background-color : rgb(255, 255, 255)')

        self.SurveyGrid_list_textbrowser = QtWidgets.QTextBrowser(self.createSurveyGrid_tab)
        self.palette = QtGui.QPalette()
        self.palette.setColor(QtGui.QPalette.Text, QtCore.Qt.black)
        self.SurveyGrid_list_textbrowser.setGeometry(QtCore.QRect(50, 160, 600, 150))
        self.SurveyGrid_list_textbrowser.setPalette(self.palette)

        self.SurveyGrid_list_textbrowser.setFont(font)
        self.SurveyGrid_list_textbrowser.setObjectName("SurveyGrid_list_textbrowser")
        self.SurveyGrid_list_textbrowser.setStyleSheet('background-color: rgb(255, 255, 255)')


        self.buttonBox_SurveyGrid_lat_lng = QtWidgets.QDialogButtonBox(self.createSurveyGrid_tab)
        self.buttonBox_SurveyGrid_lat_lng.setObjectName("buttonBox_SurveyGrid_lat_lng")
        self.buttonBox_SurveyGrid_lat_lng.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel | QtWidgets.QDialogButtonBox.Save)
        self.buttonBox_SurveyGrid_lat_lng.setGeometry(QtCore.QRect(700, 95 , 200, 40))
        #self.buttonBox_SurveyGrid_lat_lng.setStyleSheet('border-color : rgb(255, 255, 255)')
        self.buttonBox_SurveyGrid_lat_lng.setStyleSheet("background-color : rgb(20, 255, 50)")
        
        self.buttonBox_SurveyGrid_list = QtWidgets.QDialogButtonBox(self.createSurveyGrid_tab)
        self.buttonBox_SurveyGrid_list.setObjectName("buttonBox_SurveyGrid_list")
        self.buttonBox_SurveyGrid_list.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel | QtWidgets.QDialogButtonBox.Save)
        self.buttonBox_SurveyGrid_list.setGeometry(QtCore.QRect(700, 160 , 200, 40))
        self.buttonBox_SurveyGrid_list.setStyleSheet('border-color : rgb(255, 255, 255)')

        self.lineEdit_SurveyGrid_name = QtWidgets.QLineEdit(self.createSurveyGrid_tab)
        self.lineEdit_SurveyGrid_name.setObjectName("lineEdit_SurveyGrid")
        self.lineEdit_SurveyGrid_name.setGeometry(QtCore.QRect(720, 200 , 200, 40))
        self.lineEdit_SurveyGrid_name.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_SurveyGrid_name.setText("default")
        self.lineEdit_SurveyGrid_name.setAlignment(QtCore.Qt.AlignCenter)

        font = QtGui.QFont()
        font.setBold(True)
        font.setPointSize(11)
        font.setWeight(40)
        font.setBold(True)

        self.survey_info_label = QtWidgets.QLabel(self.createSurveyGrid_tab)
        self.survey_info_label.setGeometry(QtCore.QRect(10, 0, 321, 20))
        self.survey_info_label.setFont(font)
        self.survey_info_label.setTextFormat(QtCore.Qt.PlainText)
        self.survey_info_label.setAlignment(QtCore.Qt.AlignCenter)
        self.survey_info_label.setWordWrap(False)
        self.survey_info_label.setObjectName("label_info")
        self.survey_info_label.setStyleSheet('color: blue')
        self.survey_info_label.setText("White : first point - Black : last point")

        self.survey_grid_frame = QtWidgets.QFrame(self.createSurveyGrid_tab)
        self.survey_grid_frame.setGeometry(QtCore.QRect(20, 20, 950, 60))
        self.survey_grid_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.survey_grid_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.survey_grid_frame.setFrameStyle(QFrame.NoFrame)
        self.survey_grid_frame.setObjectName("frame")
        self.survey_grid_frame.setStyleSheet('background-color: rgb(0, 0, 200)')

        self.HLayoutWidget_survey = QtWidgets.QWidget(self.survey_grid_frame)
        self.HLayoutWidget_survey.setGeometry(QtCore.QRect(0, 0, 950, 60))
        self.HLayoutWidget_survey.setObjectName("gridLayoutWidget_1")

        self.HLayout_survey = QtWidgets.QHBoxLayout(self.HLayoutWidget_survey)
        self.HLayout_survey.setContentsMargins(0, 0, 0, 0)
        self.HLayout_survey.setObjectName("gridLayout")

        self.survey_angle_info_label = QtWidgets.QLabel(self.HLayoutWidget_survey)
        self.survey_angle_info_label.setAlignment(QtCore.Qt.AlignCenter)
        self.survey_angle_info_label.setWordWrap(False)
        self.survey_angle_info_label.setObjectName("label_angle_info")
        self.survey_angle_info_label.setStyleSheet('color: red')
        self.survey_angle_info_label.setText("Angle : ")

        self.HLayout_survey.addWidget(self.survey_angle_info_label)

        self.lineEdit_SurveyGrid_angle = QtWidgets.QLineEdit(self.HLayoutWidget_survey)
        self.lineEdit_SurveyGrid_angle.setObjectName("lineEdit_SurveyGrid_angle")
        self.lineEdit_SurveyGrid_angle.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_SurveyGrid_angle.setText("60")
        self.lineEdit_SurveyGrid_angle.setAlignment(QtCore.Qt.AlignCenter)

        self.HLayout_survey.addWidget(self.lineEdit_SurveyGrid_angle)

        self.survey_dist_info_label = QtWidgets.QLabel(self.HLayoutWidget_survey)
        self.survey_dist_info_label.setAlignment(QtCore.Qt.AlignCenter)
        self.survey_dist_info_label.setWordWrap(False)
        self.survey_dist_info_label.setObjectName("label_dist_info")
        self.survey_dist_info_label.setStyleSheet('color: red')
        self.survey_dist_info_label.setText("Dist : ")

        self.HLayout_survey.addWidget(self.survey_dist_info_label)

        self.lineEdit_SurveyGrid_dist = QtWidgets.QLineEdit(self.HLayoutWidget_survey)
        self.lineEdit_SurveyGrid_dist.setObjectName("lineEdit_SurveyGrid_dist")
        self.lineEdit_SurveyGrid_dist.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_SurveyGrid_dist.setText("40")
        self.lineEdit_SurveyGrid_dist.setAlignment(QtCore.Qt.AlignCenter)

        self.HLayout_survey.addWidget(self.lineEdit_SurveyGrid_dist)

        self.survey_alt_info_label = QtWidgets.QLabel(self.HLayoutWidget_survey)
        self.survey_alt_info_label.setAlignment(QtCore.Qt.AlignCenter)
        self.survey_alt_info_label.setWordWrap(False)
        self.survey_alt_info_label.setObjectName("label_dist_info")
        self.survey_alt_info_label.setStyleSheet('color: red')
        self.survey_alt_info_label.setText("Alt : ")

        self.HLayout_survey.addWidget(self.survey_alt_info_label)

        self.lineEdit_SurveyGrid_alt = QtWidgets.QLineEdit(self.HLayoutWidget_survey)
        self.lineEdit_SurveyGrid_alt.setObjectName("lineEdit_SurveyGrid_dist")
        self.lineEdit_SurveyGrid_alt.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_SurveyGrid_alt.setText("100")
        self.lineEdit_SurveyGrid_alt.setAlignment(QtCore.Qt.AlignCenter)

        self.HLayout_survey.addWidget(self.lineEdit_SurveyGrid_alt)

        self.survey_time_info_label = QtWidgets.QLabel(self.HLayoutWidget_survey)
        self.survey_time_info_label.setAlignment(QtCore.Qt.AlignCenter)
        self.survey_time_info_label.setWordWrap(False)
        self.survey_time_info_label.setObjectName("label_dist_info")
        self.survey_time_info_label.setStyleSheet('color: red')
        self.survey_time_info_label.setText("Flight Time: ")

        self.HLayout_survey.addWidget(self.survey_time_info_label)

        self.lineEdit_SurveyGrid_time = QtWidgets.QLineEdit(self.HLayoutWidget_survey)
        self.lineEdit_SurveyGrid_time.setObjectName("lineEdit_SurveyGrid_dist")
        self.lineEdit_SurveyGrid_time.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_SurveyGrid_time.setText("5")
        self.lineEdit_SurveyGrid_time.setAlignment(QtCore.Qt.AlignCenter)

        self.HLayout_survey.addWidget(self.lineEdit_SurveyGrid_time)

        self.survey_vel_info_label = QtWidgets.QLabel(self.HLayoutWidget_survey)
        self.survey_vel_info_label.setAlignment(QtCore.Qt.AlignCenter)
        self.survey_vel_info_label.setWordWrap(False)
        self.survey_vel_info_label.setObjectName("label_dist_info")
        self.survey_vel_info_label.setStyleSheet('color: red')
        self.survey_vel_info_label.setText("Average Vel: ")

        self.HLayout_survey.addWidget(self.survey_vel_info_label)

        self.lineEdit_SurveyGrid_vel = QtWidgets.QLineEdit(self.HLayoutWidget_survey)
        self.lineEdit_SurveyGrid_vel.setObjectName("lineEdit_SurveyGrid_dist")
        self.lineEdit_SurveyGrid_vel.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_SurveyGrid_vel.setText("15")
        self.lineEdit_SurveyGrid_vel.setAlignment(QtCore.Qt.AlignCenter)

        self.HLayout_survey.addWidget(self.lineEdit_SurveyGrid_vel)

        self.survey_rad_info_label = QtWidgets.QLabel(self.HLayoutWidget_survey)
        self.survey_rad_info_label.setAlignment(QtCore.Qt.AlignCenter)
        self.survey_rad_info_label.setWordWrap(False)
        self.survey_rad_info_label.setObjectName("label_dist_info")
        self.survey_rad_info_label.setStyleSheet('color: red')
        self.survey_rad_info_label.setText("Turn Radius: ")

        self.HLayout_survey.addWidget(self.survey_rad_info_label)

        self.lineEdit_SurveyGrid_rad = QtWidgets.QLineEdit(self.HLayoutWidget_survey)
        self.lineEdit_SurveyGrid_rad.setObjectName("lineEdit_SurveyGrid_dist")
        self.lineEdit_SurveyGrid_rad.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_SurveyGrid_rad.setText("20")
        self.lineEdit_SurveyGrid_rad.setAlignment(QtCore.Qt.AlignCenter)

        self.HLayout_survey.addWidget(self.lineEdit_SurveyGrid_rad)
        
        self.survey_verify_pushbutton = QtWidgets.QPushButton(self.HLayoutWidget_survey)
        self.survey_verify_pushbutton.setObjectName("verify")
        self.survey_verify_pushbutton.setStyleSheet("background-color : rgb(255, 255, 50)")
        self.survey_verify_pushbutton.setText("Verify")

        self.HLayout_survey.addWidget(self.survey_verify_pushbutton)

        ###############################################################
        self.overlap_frame = QtWidgets.QFrame(self.createSurveyGrid_tab)
        self.overlap_frame .setGeometry(QtCore.QRect(50, 330, 600, 111))
        self.overlap_frame .setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.overlap_frame .setFrameShadow(QtWidgets.QFrame.Raised)
        self.overlap_frame .setFrameStyle(QFrame.NoFrame)
        self.overlap_frame .setObjectName("frame")
        self.overlap_frame.setStyleSheet('background-color: rgb(99, 184, 255)')

        self.gridLayoutWidget_overlap = QtWidgets.QWidget(self.overlap_frame)
        self.gridLayoutWidget_overlap.setGeometry(QtCore.QRect(0, 0, 600, 111))
        self.gridLayoutWidget_overlap.setObjectName("gridLayoutWidget_1")

        self.gridLayout_overlap = QtWidgets.QGridLayout(self.gridLayoutWidget_overlap)
        self.gridLayout_overlap.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_overlap.setObjectName("gridLayout")

        self.overlap_info_label_1 = QtWidgets.QLabel(self.gridLayoutWidget_overlap)
        self.overlap_info_label_1.setAlignment(QtCore.Qt.AlignCenter)
        self.overlap_info_label_1.setWordWrap(False)
        self.overlap_info_label_1.setObjectName("label_angle_info")
        self.overlap_info_label_1.setText("Overlap Calculator")

        self.gridLayout_overlap.addWidget(self.overlap_info_label_1, 0, 0, 1, 1)

        self.overlap_info_label_2 = QtWidgets.QLabel(self.gridLayoutWidget_overlap)
        self.overlap_info_label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.overlap_info_label_2.setWordWrap(False)
        self.overlap_info_label_2.setObjectName("label_angle_info")
        self.overlap_info_label_2.setText("Distance (m)")

        self.gridLayout_overlap.addWidget(self.overlap_info_label_2, 1, 0, 1, 1)

        self.overlap_info_label_3 = QtWidgets.QLabel(self.gridLayoutWidget_overlap)
        self.overlap_info_label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.overlap_info_label_3.setWordWrap(False)
        self.overlap_info_label_3.setObjectName("label_angle_info")
        self.overlap_info_label_3.setText("Focal Lenght (mm)")

        self.gridLayout_overlap.addWidget(self.overlap_info_label_3, 1, 1, 1, 1)

        self.overlap_info_label_4 = QtWidgets.QLabel(self.gridLayoutWidget_overlap)
        self.overlap_info_label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.overlap_info_label_4.setWordWrap(False)
        self.overlap_info_label_4.setObjectName("label_angle_info")
        self.overlap_info_label_4.setText("Altitude (m)")

        self.gridLayout_overlap.addWidget(self.overlap_info_label_4, 1, 2, 1, 1)

        self.overlap_info_label_5 = QtWidgets.QLabel(self.gridLayoutWidget_overlap)
        self.overlap_info_label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.overlap_info_label_5.setWordWrap(False)
        self.overlap_info_label_5.setObjectName("label_angle_info")
        self.overlap_info_label_5.setText("Widht (mm)")

        self.gridLayout_overlap.addWidget(self.overlap_info_label_5, 1, 3, 1, 1)

        self.overlap_info_label_6 = QtWidgets.QLabel(self.gridLayoutWidget_overlap)
        self.overlap_info_label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.overlap_info_label_6.setWordWrap(False)
        self.overlap_info_label_6.setObjectName("label_angle_info")
        self.overlap_info_label_6.setText("Percentage")

        self.gridLayout_overlap.addWidget(self.overlap_info_label_6, 1, 4, 1, 1)


        self.lineEdit_overlap_1 = QtWidgets.QLineEdit(self.gridLayoutWidget_overlap)
        self.lineEdit_overlap_1.setObjectName("lineEdit_SurveyGrid")
        self.lineEdit_overlap_1.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_overlap_1.setAlignment(QtCore.Qt.AlignCenter)

        self.gridLayout_overlap.addWidget(self.lineEdit_overlap_1, 2, 0, 1, 1)

        self.lineEdit_overlap_2 = QtWidgets.QLineEdit(self.gridLayoutWidget_overlap)
        self.lineEdit_overlap_2.setObjectName("lineEdit_SurveyGrid")
        self.lineEdit_overlap_2.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_overlap_2.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit_overlap_2.setText("25")

        self.gridLayout_overlap.addWidget(self.lineEdit_overlap_2, 2, 1, 1, 1)

        self.lineEdit_overlap_3 = QtWidgets.QLineEdit(self.gridLayoutWidget_overlap)
        self.lineEdit_overlap_3.setObjectName("lineEdit_SurveyGrid")
        self.lineEdit_overlap_3.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_overlap_3.setAlignment(QtCore.Qt.AlignCenter)

        self.gridLayout_overlap.addWidget(self.lineEdit_overlap_3, 2, 2, 1, 1)

        self.lineEdit_overlap_4 = QtWidgets.QLineEdit(self.gridLayoutWidget_overlap)
        self.lineEdit_overlap_4.setObjectName("lineEdit_SurveyGrid")
        self.lineEdit_overlap_4.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_overlap_4.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit_overlap_4.setText("66.3")
        self.gridLayout_overlap.addWidget(self.lineEdit_overlap_4, 2, 3, 1, 1)

        self.lineEdit_overlap_5 = QtWidgets.QLineEdit(self.gridLayoutWidget_overlap)
        self.lineEdit_overlap_5.setObjectName("lineEdit_SurveyGrid")
        self.lineEdit_overlap_5.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_overlap_5.setAlignment(QtCore.Qt.AlignCenter)

        self.gridLayout_overlap.addWidget(self.lineEdit_overlap_5, 2, 4, 1, 1)

        self.overlap_calculate_pushbutton = QtWidgets.QPushButton(self.gridLayoutWidget_overlap)
        self.overlap_calculate_pushbutton.setObjectName("cal")
        self.overlap_calculate_pushbutton.setStyleSheet("background-color : rgb(255, 255, 50)")
        self.overlap_calculate_pushbutton.setText("Calculate")

        self.gridLayout_overlap.addWidget(self.overlap_calculate_pushbutton, 0, 4, 1, 1)

    def editMission_(self):
        self.lineEdit_edit_name = QtWidgets.QLineEdit(self.editMission_tab)
        self.lineEdit_edit_name.setObjectName("lineEdit_createPlan_alt")
        self.lineEdit_edit_name.setGeometry(QtCore.QRect(20, 20 , 200, 40))
        self.lineEdit_edit_name.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_edit_name.setText("default.txt")
        self.lineEdit_edit_name.setAlignment(QtCore.Qt.AlignCenter)

        self.lineEdit_write_name = QtWidgets.QLineEdit(self.editMission_tab)
        self.lineEdit_write_name.setObjectName("lineEdit_createPlan_alt")
        self.lineEdit_write_name.setGeometry(QtCore.QRect(400, 20 , 200, 40))
        self.lineEdit_write_name.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_write_name.setText("default_w.txt")
        self.lineEdit_write_name.setAlignment(QtCore.Qt.AlignCenter)

        self.lon_edit_name = QtWidgets.QLineEdit(self.editMission_tab)
        self.lon_edit_name.setObjectName("lineEdit_createPlan_alt")
        self.lon_edit_name.setGeometry(QtCore.QRect(710, 20 , 80, 40))
        self.lon_edit_name.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lon_edit_name.setText("-")

        self.lng_edit_name = QtWidgets.QLineEdit(self.editMission_tab)
        self.lng_edit_name.setObjectName("lineEdit_createPlan_alt")
        self.lng_edit_name.setGeometry(QtCore.QRect(810, 20 , 80, 40))
        self.lng_edit_name.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lng_edit_name.setText("-")

        self.writeMission_pushbutton = QtWidgets.QPushButton(self.editMission_tab)
        font = QtGui.QFont()
        font.setUnderline(True)
        self.writeMission_pushbutton.setFont(font)
        self.writeMission_pushbutton.setObjectName("Write")
        self.writeMission_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.writeMission_pushbutton.setGeometry(QtCore.QRect(610, 25, 93, 28))
        self.writeMission_pushbutton.setText("Write")

        self.clearMission_pushbutton = QtWidgets.QPushButton(self.editMission_tab)
        font = QtGui.QFont()
        font.setUnderline(True)
        self.clearMission_pushbutton.setFont(font)
        self.clearMission_pushbutton.setObjectName("Clear")
        self.clearMission_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.clearMission_pushbutton.setGeometry(QtCore.QRect(220, 400, 93, 28))
        self.clearMission_pushbutton.setText("Clear")


        self.readMission_pushbutton = QtWidgets.QPushButton(self.editMission_tab)
        font = QtGui.QFont()
        font.setUnderline(True)
        self.readMission_pushbutton.setFont(font)
        self.readMission_pushbutton.setObjectName("Read")
        self.readMission_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.readMission_pushbutton.setGeometry(QtCore.QRect(230, 25, 93, 28))
        self.readMission_pushbutton.setText("Read")


        self.moveup_pushbutton = QtWidgets.QPushButton(self.editMission_tab)
        font = QtGui.QFont()
        font.setUnderline(True)
        self.moveup_pushbutton.setFont(font)
        self.moveup_pushbutton.setObjectName("Moveu")
        self.moveup_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.moveup_pushbutton.setGeometry(QtCore.QRect(20, 400, 93, 28))
        self.moveup_pushbutton.setText("Move up")

        self.movedown_pushbutton = QtWidgets.QPushButton(self.editMission_tab)
        font = QtGui.QFont()
        font.setUnderline(True)
        self.movedown_pushbutton.setFont(font)
        self.movedown_pushbutton.setObjectName("Moved")
        self.movedown_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.movedown_pushbutton.setGeometry(QtCore.QRect(110, 400, 93, 28))
        self.movedown_pushbutton.setText("Move down")

        self.delete_pushbutton = QtWidgets.QPushButton(self.editMission_tab)
        font = QtGui.QFont()
        font.setUnderline(True)
        self.delete_pushbutton.setFont(font)
        self.delete_pushbutton.setObjectName("Moved")
        self.delete_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.delete_pushbutton.setGeometry(QtCore.QRect(700, 400, 93, 28))
        self.delete_pushbutton.setText("Remove")

        self.lineEdit_delete_name = QtWidgets.QLineEdit(self.editMission_tab)
        self.lineEdit_delete_name.setObjectName("lineEdit_createPlan_alt")
        self.lineEdit_delete_name.setGeometry(QtCore.QRect(800, 400, 93, 28))
        self.lineEdit_delete_name.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_delete_name.setText("1")
        self.lineEdit_delete_name.setAlignment(QtCore.Qt.AlignCenter)

        self.tableWidget = QTableWidget(self.editMission_tab)
        self.tableWidget.setGeometry(QtCore.QRect(10, 100, 980, 300))
        self.tableWidget.setStyleSheet('background-color : rgb(255, 255, 255)')

        #Row count
        self.tableWidget.setRowCount(100) 
  
        #Column count
        self.tableWidget.setColumnCount(12)  
  
        self.tableWidget.setHorizontalHeaderItem(0, QTableWidgetItem("seq"))
        self.tableWidget.setHorizontalHeaderItem(1, QTableWidgetItem("current"))
        self.tableWidget.setHorizontalHeaderItem(2, QTableWidgetItem("frame"))
        self.tableWidget.setHorizontalHeaderItem(3, QTableWidgetItem("command"))
        self.tableWidget.setHorizontalHeaderItem(4, QTableWidgetItem("param1"))
        self.tableWidget.setHorizontalHeaderItem(5, QTableWidgetItem("param2"))
        self.tableWidget.setHorizontalHeaderItem(6, QTableWidgetItem("param3"))
        self.tableWidget.setHorizontalHeaderItem(7, QTableWidgetItem("param4"))
        self.tableWidget.setHorizontalHeaderItem(8, QTableWidgetItem("x"))
        self.tableWidget.setHorizontalHeaderItem(9, QTableWidgetItem("y"))
        self.tableWidget.setHorizontalHeaderItem(10, QTableWidgetItem("z"))
        self.tableWidget.setHorizontalHeaderItem(11, QTableWidgetItem("autocontinue"))
        #Table will fit the screen horizontally
        self.tableWidget.horizontalHeader().setStretchLastSection(True)
        self.tableWidget.horizontalHeader().setSectionResizeMode(
           QHeaderView.Stretch)    

    def loadMission_(self):
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setWeight(10)
        font.setBold(True)
        
        self.lineEdit_mission_name = QtWidgets.QLineEdit(self.loadMission_tab)
        self.lineEdit_mission_name.setObjectName("lineEdit_createPlan_alt")
        self.lineEdit_mission_name.setGeometry(QtCore.QRect(20, 20 , 200, 40))
        self.lineEdit_mission_name.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_mission_name.setText("default.txt")
        self.lineEdit_mission_name.setAlignment(QtCore.Qt.AlignCenter)

        self.plan_textbrowser_load = QtWidgets.QTextBrowser(self.loadMission_tab)
        self.palette = QtGui.QPalette()
        self.palette.setColor(QtGui.QPalette.Text, QtCore.Qt.black)
        self.plan_textbrowser_load.setGeometry(QtCore.QRect(10, 120, 980, 300))
        self.plan_textbrowser_load.setPalette(self.palette)

        self.plan_textbrowser_load.setObjectName("plan_textBrowser_2")
        self.plan_textbrowser_load.setStyleSheet('background-color: rgb(255, 255, 255)')
        
        self.plan_textbrowser_load.setFont(font)
        self.buttonBox_load = QtWidgets.QDialogButtonBox(self.loadMission_tab)
        self.buttonBox_load.setObjectName("buttonBox_createPlan")
        self.buttonBox_load.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel | QtWidgets.QDialogButtonBox.Save)
        self.buttonBox_load.setGeometry(QtCore.QRect(8, 80 , 200, 40))
        self.buttonBox_load.setStyleSheet('border-color : rgb(255, 255, 255)')
        
    def createPlan_(self):
        font = QtGui.QFont()
        font.setBold(True)
        font.setPointSize(9)
        font.setWeight(20)

        self.createPlan_tab.setStyleSheet('background-color : rgb(0, 255, 255)')
        self.lineEdit_createPlan = QtWidgets.QLineEdit(self.createPlan_tab)
        self.lineEdit_createPlan.setObjectName("lineEdit_createPlan")
        self.lineEdit_createPlan.setGeometry(QtCore.QRect(50, 200, 600, 30))
        self.lineEdit_createPlan.setStyleSheet('background-color : rgb(255, 255, 255)')

        self.lineEdit_createPlan_command = QtWidgets.QLineEdit(self.createPlan_tab)
        self.lineEdit_createPlan_command.setObjectName("lineEdit_createPlan_c3mmand")
        self.lineEdit_createPlan_command.setGeometry(QtCore.QRect(650, 200, 50, 30))
        self.lineEdit_createPlan_command.setStyleSheet('background-color : rgb(255, 255, 255)')

        self.lineEdit_createPlan_alt = QtWidgets.QLineEdit(self.createPlan_tab)
        self.lineEdit_createPlan_alt.setObjectName("lineEdit_createPlan_alt")
        self.lineEdit_createPlan_alt.setGeometry(QtCore.QRect(700, 200, 50, 30))
        self.lineEdit_createPlan_alt.setStyleSheet('background-color : rgb(255, 255, 255)')

        self.lineEdit_createPlan_name = QtWidgets.QLineEdit(self.createPlan_tab)
        self.lineEdit_createPlan_name.setObjectName("lineEdit_createPlan_alt")
        self.lineEdit_createPlan_name.setGeometry(QtCore.QRect(720, 300 , 200, 40))
        self.lineEdit_createPlan_name.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_createPlan_name.setText("default.py")
        self.lineEdit_createPlan_name.setAlignment(QtCore.Qt.AlignCenter)

        self.lineEdit_marker_lat = QtWidgets.QLineEdit(self.createPlan_tab)
        self.lineEdit_marker_lat.setObjectName("lineEdit_createPlan_alt")
        self.lineEdit_marker_lat.setGeometry(QtCore.QRect(650, 50, 150, 30))
        self.lineEdit_marker_lat.setStyleSheet('background-color : rgb(255, 255, 255)')

        self.lineEdit_marker_lon = QtWidgets.QLineEdit(self.createPlan_tab)
        self.lineEdit_marker_lon.setObjectName("lineEdit_createPlan_alt")
        self.lineEdit_marker_lon.setGeometry(QtCore.QRect(800, 50, 150, 30))
        self.lineEdit_marker_lon.setStyleSheet('background-color : rgb(255, 255, 255)')

        self.buttonBox_createmarker = QtWidgets.QDialogButtonBox(self.createPlan_tab)
        self.buttonBox_createmarker.setObjectName("buttonBox_createPlan")
        self.buttonBox_createmarker.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel | QtWidgets.QDialogButtonBox.Save)
        self.buttonBox_createmarker.setGeometry(QtCore.QRect(680, 90 , 200, 40))
        self.buttonBox_createmarker.setStyleSheet('border-color : rgb(255, 255, 255)')

        self.buttonBox_createPlan = QtWidgets.QDialogButtonBox(self.createPlan_tab)
        self.buttonBox_createPlan.setObjectName("buttonBox_createPlan")
        self.buttonBox_createPlan.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel | QtWidgets.QDialogButtonBox.Save)
        self.buttonBox_createPlan.setGeometry(QtCore.QRect(760, 195 , 200, 40))
        self.buttonBox_createPlan.setStyleSheet('border-color : rgb(255, 255, 255)')
        
        self.plan_textbrowser = QtWidgets.QTextBrowser(self.createPlan_tab)
        self.palette = QtGui.QPalette()
        self.palette.setColor(QtGui.QPalette.Text, QtCore.Qt.black)
        self.plan_textbrowser.setGeometry(QtCore.QRect(350, 20, 230, 150))
        self.plan_textbrowser.setPalette(self.palette)

        self.plan_textbrowser.setFont(font)
        self.plan_textbrowser.setObjectName("plan_textBrowser_2")
        self.plan_textbrowser.setStyleSheet('background-color: rgb(255, 255, 255)')

        self.plan_textbrowser.append("uint16 command\
                                        uint16 NAV_WAYPOINT = 16\
                                        uint16 NAV_LOITER_UNLIM = 17\
                                        uint16 NAV_LOITER_TURNS = 18\
                                        uint16 NAV_LOITER_TIME = 19\
                                        uint16 NAV_RETURN_TO_LAUNCH = 20\
                                        uint16 NAV_LAND = 21\
                                        uint16 NAV_TAKEOFF = 22\
                                        ")

        self.plan_textbrowser_2 = QtWidgets.QTextBrowser(self.createPlan_tab)
        self.palette = QtGui.QPalette()
        self.palette.setColor(QtGui.QPalette.Text, QtCore.Qt.black)
        self.plan_textbrowser_2.setGeometry(QtCore.QRect(50, 260, 600, 150))
        self.plan_textbrowser_2.setPalette(self.palette)

        self.plan_textbrowser_2.setFont(font)
        self.plan_textbrowser_2.setObjectName("plan_textBrowser_2")
        self.plan_textbrowser_2.setStyleSheet('background-color: rgb(255, 255, 255)')

        self.buttonBox_createPlan_2 = QtWidgets.QDialogButtonBox(self.createPlan_tab)
        self.buttonBox_createPlan_2.setObjectName("buttonBox_createPlan")
        self.buttonBox_createPlan_2.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel | QtWidgets.QDialogButtonBox.Save)
        self.buttonBox_createPlan_2.setGeometry(QtCore.QRect(700, 260 , 200, 40))
        self.buttonBox_createPlan_2.setStyleSheet('border-color : rgb(255, 255, 255)')
        
    def createPlan_A(self):
        font = QtGui.QFont()
        font.setBold(True)
        font.setPointSize(9)
        font.setWeight(20)

        self.createPlanA_tab.setStyleSheet('background-color : rgb(255, 0, 255)')
        self.lineEdit_createPlanA_tab = QtWidgets.QLineEdit(self.createPlanA_tab)
        self.lineEdit_createPlanA_tab.setObjectName("lineEdit_createPlan")
        self.lineEdit_createPlanA_tab.setGeometry(QtCore.QRect(50, 200, 600, 30))
        self.lineEdit_createPlanA_tab.setStyleSheet('background-color : rgb(255, 255, 255)')

        self.lineEdit_createPlanA_tab_command = QtWidgets.QLineEdit(self.createPlanA_tab)
        self.lineEdit_createPlanA_tab_command.setObjectName("lineEdit_createPlan_command")
        self.lineEdit_createPlanA_tab_command.setGeometry(QtCore.QRect(650, 200, 50, 30))
        self.lineEdit_createPlanA_tab_command.setStyleSheet('background-color : rgb(255, 255, 255)')

        self.lineEdit_createPlanA_tab_alt = QtWidgets.QLineEdit(self.createPlanA_tab)
        self.lineEdit_createPlanA_tab_alt.setObjectName("lineEdit_createPlan_alt")
        self.lineEdit_createPlanA_tab_alt.setGeometry(QtCore.QRect(700, 200, 50, 30))
        self.lineEdit_createPlanA_tab_alt.setStyleSheet('background-color : rgb(255, 255, 255)')

        self.lineEdit_createPlanA_tab_name = QtWidgets.QLineEdit(self.createPlanA_tab)
        self.lineEdit_createPlanA_tab_name.setObjectName("lineEdit_createPlan_alt")
        self.lineEdit_createPlanA_tab_name.setGeometry(QtCore.QRect(720, 300 , 200, 40))
        self.lineEdit_createPlanA_tab_name.setStyleSheet('background-color : rgb(255, 255, 255)')
        self.lineEdit_createPlanA_tab_name.setText("default.py")
        self.lineEdit_createPlanA_tab_name.setAlignment(QtCore.Qt.AlignCenter)

        self.lineEdit_marker_lat_A = QtWidgets.QLineEdit(self.createPlanA_tab)
        self.lineEdit_marker_lat_A.setObjectName("lineEdit_createPlan_alt")
        self.lineEdit_marker_lat_A.setGeometry(QtCore.QRect(650, 50, 150, 30))
        self.lineEdit_marker_lat_A.setStyleSheet('background-color : rgb(255, 255, 255)')

        self.lineEdit_marker_lon_A = QtWidgets.QLineEdit(self.createPlanA_tab)
        self.lineEdit_marker_lon_A.setObjectName("lineEdit_createPlan_alt")
        self.lineEdit_marker_lon_A.setGeometry(QtCore.QRect(800, 50, 150, 30))
        self.lineEdit_marker_lon_A.setStyleSheet('background-color : rgb(255, 255, 255)')

        self.buttonBox_createmarker_A = QtWidgets.QDialogButtonBox(self.createPlanA_tab)
        self.buttonBox_createmarker_A.setObjectName("buttonBox_createPlan")
        self.buttonBox_createmarker_A.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel | QtWidgets.QDialogButtonBox.Save)
        self.buttonBox_createmarker_A.setGeometry(QtCore.QRect(680, 90 , 200, 40))
        self.buttonBox_createmarker_A.setStyleSheet('border-color : rgb(255, 255, 255)')

        self.buttonBox_createPlan_A = QtWidgets.QDialogButtonBox(self.createPlanA_tab)
        self.buttonBox_createPlan_A.setObjectName("buttonBox_createPlan")
        self.buttonBox_createPlan_A.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel | QtWidgets.QDialogButtonBox.Save)
        self.buttonBox_createPlan_A.setGeometry(QtCore.QRect(760, 195 , 200, 40))
        self.buttonBox_createPlan_A.setStyleSheet('border-color : rgb(255, 255, 255)')
        
        self.plan_textbrowser_A = QtWidgets.QTextBrowser(self.createPlanA_tab)
        self.palette = QtGui.QPalette()
        self.palette.setColor(QtGui.QPalette.Text, QtCore.Qt.black)
        self.plan_textbrowser_A.setGeometry(QtCore.QRect(350, 20, 230, 150))
        self.plan_textbrowser_A.setPalette(self.palette)

        self.plan_textbrowser_A.setFont(font)
        self.plan_textbrowser_A.setObjectName("plan_textBrowser_2")
        self.plan_textbrowser_A.setStyleSheet('background-color: rgb(255, 255, 255)')

        self.plan_textbrowser_A.append("uint16 command\
                                        uint16 NAV_WAYPOINT = 16\
                                        uint16 NAV_LOITER_UNLIM = 17\
                                        uint16 NAV_LOITER_TURNS = 18\
                                        uint16 NAV_LOITER_TIME = 19\
                                        uint16 NAV_RETURN_TO_LAUNCH = 20\
                                        uint16 NAV_LAND = 21\
                                        uint16 NAV_TAKEOFF = 22\
                                        ")

        self.plan_textbrowser_2_A = QtWidgets.QTextBrowser(self.createPlanA_tab)
        self.palette = QtGui.QPalette()
        self.palette.setColor(QtGui.QPalette.Text, QtCore.Qt.black)
        self.plan_textbrowser_2_A.setGeometry(QtCore.QRect(50, 260, 600, 150))
        self.plan_textbrowser_2_A.setPalette(self.palette)

        self.plan_textbrowser_2_A.setFont(font)
        self.plan_textbrowser_2_A.setObjectName("plan_textBrowser_2")
        self.plan_textbrowser_2_A.setStyleSheet('background-color: rgb(255, 255, 255)')

        self.buttonBox_createPlan_2_A = QtWidgets.QDialogButtonBox(self.createPlanA_tab)
        self.buttonBox_createPlan_2_A.setObjectName("buttonBox_createPlan")
        self.buttonBox_createPlan_2_A.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel | QtWidgets.QDialogButtonBox.Save)
        self.buttonBox_createPlan_2_A.setGeometry(QtCore.QRect(700, 260 , 200, 40))
        self.buttonBox_createPlan_2_A.setStyleSheet('border-color : rgb(255, 255, 255)')

    def lcdnumber_(self):
        self.heading_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_1)
        self.heading_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.heading_lcdNumber.setDigitCount(10)
        self.heading_lcdNumber.setProperty("value", 0)
        self.heading_lcdNumber.setObjectName("lcdNumber_5")
        self.gridLayout_1.addWidget(self.heading_lcdNumber, 4, 1, 1, 1)

        self.barheight_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_1)
        self.barheight_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.barheight_lcdNumber.setDigitCount(10)
        self.barheight_lcdNumber.setProperty("value", 0)
        self.barheight_lcdNumber.setObjectName("lcdNumber_4")
        self.gridLayout_1.addWidget(self.barheight_lcdNumber, 3, 1, 1, 1)

        self.altitude_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_1)
        self.altitude_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.altitude_lcdNumber.setDigitCount(10)
        self.altitude_lcdNumber.setProperty("value", 0)
        self.altitude_lcdNumber.setObjectName("lcdNumber")
        self.gridLayout_1.addWidget(self.altitude_lcdNumber, 0, 1, 1, 1)

        self.latitude_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_1)
        self.latitude_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.latitude_lcdNumber.setDigitCount(10)
        self.latitude_lcdNumber.setProperty("value", 0)
        self.latitude_lcdNumber.setObjectName("lcdNumber_2")
        self.gridLayout_1.addWidget(self.latitude_lcdNumber, 1, 1, 1, 1)

        self.longitude_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_1)
        self.longitude_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.longitude_lcdNumber.setDigitCount(10)
        self.longitude_lcdNumber.setProperty("value", 0)
        self.longitude_lcdNumber.setObjectName("lcdNumber_3")
        self.gridLayout_1.addWidget(self.longitude_lcdNumber, 2, 1, 1, 1)


        self.roll_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_2)
        self.roll_lcdNumber.setDigitCount(10)
        self.roll_lcdNumber.setProperty("value", 0)
        self.roll_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.roll_lcdNumber.setObjectName("lcdNumber_6")
        self.gridLayout_2.addWidget(self.roll_lcdNumber, 0, 1, 1, 1)

        self.pitch_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_2)
        self.pitch_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.pitch_lcdNumber.setDigitCount(10)
        self.pitch_lcdNumber.setProperty("value", 0)
        self.pitch_lcdNumber.setObjectName("lcdNumber_7")
        self.gridLayout_2.addWidget(self.pitch_lcdNumber, 1, 1, 1, 1)

        self.yaw_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_2)
        self.yaw_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.yaw_lcdNumber.setDigitCount(10)
        self.yaw_lcdNumber.setProperty("value", 0)
        self.yaw_lcdNumber.setObjectName("lcdNumber_8")
        self.gridLayout_2.addWidget(self.yaw_lcdNumber, 2, 1, 1, 1)

        self.speedx_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_2)
        self.speedx_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.speedx_lcdNumber.setDigitCount(10)
        self.speedx_lcdNumber.setProperty("value", 0)
        self.speedx_lcdNumber.setObjectName("lcdNumber_9")
        self.gridLayout_2.addWidget(self.speedx_lcdNumber, 3, 1, 1, 1)

        self.speedy_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_2)
        self.speedy_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.speedy_lcdNumber.setDigitCount(10)
        self.speedy_lcdNumber.setProperty("value", 0)
        self.speedy_lcdNumber.setObjectName("lcdNumber_10")
        self.gridLayout_2.addWidget(self.speedy_lcdNumber, 4, 1, 1, 1)

        self.speedz_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_2)
        self.speedz_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.speedz_lcdNumber.setDigitCount(10)
        self.speedz_lcdNumber.setProperty("value", 0)
        self.speedz_lcdNumber.setObjectName("lcdNumber_11")
        self.gridLayout_2.addWidget(self.speedz_lcdNumber, 5, 1, 1, 1)

        self.mission1time_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_5)
        self.mission1time_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.mission1time_lcdNumber.setDigitCount(10)
        self.mission1time_lcdNumber.display('0:0:0')
        self.mission1time_lcdNumber.setObjectName("lcdNumber_12")
        self.gridLayout_5.addWidget(self.mission1time_lcdNumber, 0, 1, 1, 1)

        self.mission2time_lcdNumber = QtWidgets.QLCDNumber(self.gridLayoutWidget_7)
        self.mission2time_lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.mission2time_lcdNumber.setDigitCount(10)
        self.mission2time_lcdNumber.display('0:0:0')
        self.mission2time_lcdNumber.setObjectName("lcdNumber_13")
        self.gridLayout_7.addWidget(self.mission2time_lcdNumber, 0, 1, 1, 1)


    def label_(self):
        font = QtGui.QFont()
        font.setBold(True)
        font.setPointSize(11)
        font.setWeight(40)
        font.setBold(True)

        self.uav_localization_label = QtWidgets.QLabel(self.main_frame_right)
        self.uav_localization_label.setGeometry(QtCore.QRect(90, 30, 271, 20))
        self.uav_localization_label.setFont(font)
        self.uav_localization_label.setTextFormat(QtCore.Qt.PlainText)
        self.uav_localization_label.setAlignment(QtCore.Qt.AlignCenter)
        self.uav_localization_label.setWordWrap(False)
        self.uav_localization_label.setObjectName("label_1")
        self.uav_localization_label.setStyleSheet('color: red')

        self.created = QtWidgets.QLabel(self.main_frame_right)
        self.created.setGeometry(QtCore.QRect(100, 900, 271, 20))
        self.created.setFont(font)
        self.created.setTextFormat(QtCore.Qt.PlainText)
        self.created.setAlignment(QtCore.Qt.AlignRight)
        self.created.setWordWrap(False)
        self.created.setObjectName("label_121")
        self.created.setStyleSheet('color: red')

        font = QtGui.QFont()
        font.setPointSize(11)
        font.setWeight(50)
        font.setBold(True)

        self.uav_imudata_label = QtWidgets.QLabel(self.main_frame_right)
        self.uav_imudata_label.setGeometry(QtCore.QRect(145, 400, 141, 21))
        self.uav_imudata_label.setFont(font)
        self.uav_imudata_label.setAlignment(QtCore.Qt.AlignCenter)
        self.uav_imudata_label.setObjectName("label_7")
        self.uav_imudata_label.setStyleSheet('color: red')


        self.uav_status_label = QtWidgets.QLabel(self.main_frame_right)
        self.uav_status_label.setFont(font)
        self.uav_status_label.setTextFormat(QtCore.Qt.PlainText)
        self.uav_status_label.setGeometry(QtCore.QRect(140, 740, 141, 21))
        self.uav_status_label.setAlignment(QtCore.Qt.AlignCenter)
        self.uav_status_label.setObjectName("label_14")
        self.uav_status_label.setStyleSheet('color: red')

        font = QtGui.QFont()
        font.setPointSize(12)
        font.setWeight(30)
        font.setBold(True)

        self.longitude_label = QtWidgets.QLabel(self.gridLayoutWidget_1)
        self.longitude_label.setFont(font)
        self.longitude_label.setObjectName("label_4")
        self.gridLayout_1.addWidget(self.longitude_label, 2, 0, 1, 1)

        self.altitude_label = QtWidgets.QLabel(self.gridLayoutWidget_1)
        self.altitude_label.setFont(font)
        self.altitude_label.setObjectName("label_2")
        self.gridLayout_1.addWidget(self.altitude_label, 0, 0, 1, 1)

        self.bar_height_label = QtWidgets.QLabel(self.gridLayoutWidget_1)
        self.bar_height_label.setFont(font)
        self.bar_height_label.setObjectName("label_5")
        self.gridLayout_1.addWidget(self.bar_height_label, 3, 0, 1, 1)

        self.heading_label = QtWidgets.QLabel(self.gridLayoutWidget_1)
        self.heading_label.setFont(font)
        self.heading_label.setObjectName("label_6")
        self.gridLayout_1.addWidget(self.heading_label, 4, 0, 1, 1)

        self.latitude_label = QtWidgets.QLabel(self.gridLayoutWidget_1)
        self.latitude_label.setFont(font)
        self.latitude_label.setObjectName("label_3")
        self.gridLayout_1.addWidget(self.latitude_label, 1, 0, 1, 1)

        self.speed_z_label = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.speed_z_label.setObjectName("label_8")
        self.speed_z_label.setFont(font)
        self.gridLayout_2.addWidget(self.speed_z_label, 5, 0, 1, 1)

        self.speed_y_label = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.speed_y_label.setFont(font)
        self.speed_y_label.setObjectName("label_9")
        self.gridLayout_2.addWidget(self.speed_y_label, 4, 0, 1, 1)

        self.speed_x_label = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.speed_x_label.setFont(font)
        self.speed_x_label.setObjectName("label_10")
        self.gridLayout_2.addWidget(self.speed_x_label, 3, 0, 1, 1)

        self.pitch_label = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.pitch_label.setFont(font)
        self.pitch_label.setObjectName("label_12")
        self.gridLayout_2.addWidget(self.pitch_label, 1, 0, 1, 1)

        self.yaw_label = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.yaw_label.setFont(font)
        self.yaw_label.setObjectName("label_11")
        self.gridLayout_2.addWidget(self.yaw_label, 2, 0, 1, 1)

        self.roll_label = QtWidgets.QLabel(self.gridLayoutWidget_2)
        self.roll_label.setFont(font)
        self.roll_label.setObjectName("label_13")
        self.gridLayout_2.addWidget(self.roll_label, 0, 0, 1, 1)

        self.flying_label = QtWidgets.QLabel(self.gridLayoutWidget_3)
        self.flying_label.setFont(font)
        self.flying_label.setObjectName("label_17")
        self.gridLayout_3.addWidget(self.flying_label, 0, 0, 1, 1)

        self.battery_current_label = QtWidgets.QLabel(self.gridLayoutWidget_3)
        self.battery_current_label.setFont(font)
        self.battery_current_label.setObjectName("label_15")
        self.gridLayout_3.addWidget(self.battery_current_label, 1, 0, 1, 1)

        self.flying_status_label = QtWidgets.QLabel(self.gridLayoutWidget_3)
        self.flying_status_label.setAlignment(QtCore.Qt.AlignCenter)
        self.flying_status_label.setObjectName("label_26")
        self.gridLayout_3.addWidget(self.flying_status_label, 0, 1, 1, 2)

        self.battery_currentstatus_label = QtWidgets.QLabel(self.gridLayoutWidget_3)
        self.battery_currentstatus_label.setAlignment(QtCore.Qt.AlignCenter)
        self.battery_currentstatus_label.setObjectName("label_18")
        self.gridLayout_3.addWidget(self.battery_currentstatus_label, 1, 1, 1, 2)

        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)

        # self.console_label = QtWidgets.QLabel(self.console_frame)
        # self.console_label.setGeometry(QtCore.QRect(150, 10, 151, 20))
        # self.console_label.setFont(font)
        # self.console_label.setFrameShadow(QtWidgets.QFrame.Plain)
        # self.console_label.setAlignment(QtCore.Qt.AlignCenter)
        # self.console_label.setObjectName("label_25")
        # self.console_label.setText("CONSOLE")
        # self.console_label.setStyleSheet('color: red')

        self.mission1_label = QtWidgets.QLabel(self.gridLayoutWidget_4)
        self.mission1_label.setObjectName("label_19")
        self.gridLayout_4.addWidget(self.mission1_label, 0, 0, 1, 1)

        self.control1_panel_label = QtWidgets.QLabel(self.gridLayoutWidget_4)
        self.control1_panel_label.setObjectName("label_20")
        self.gridLayout_4.addWidget(self.control1_panel_label, 1, 0, 1, 1)

        self.mission1_time_label = QtWidgets.QLabel(self.gridLayoutWidget_5)
        self.mission1_time_label.setObjectName("label_21")
        self.gridLayout_5.addWidget(self.mission1_time_label, 0, 0, 1, 1)

        self.mission2_label = QtWidgets.QLabel(self.gridLayoutWidget_6)
        self.mission2_label.setObjectName("label_22")
        self.gridLayout_6.addWidget(self.mission2_label, 0, 0, 1, 1)

        self.control2_panel_label = QtWidgets.QLabel(self.gridLayoutWidget_6)
        self.control2_panel_label.setObjectName("label_23")
        self.gridLayout_6.addWidget(self.control2_panel_label, 1, 0, 1, 1)

        self.mission2_time_label = QtWidgets.QLabel(self.gridLayoutWidget_7)
        self.mission2_time_label.setObjectName("label_24")
        self.gridLayout_7.addWidget(self.mission2_time_label, 0, 0, 1, 1)

        font = QtGui.QFont()
        font.setBold(True)
        font.setPointSize(11)
        font.setWeight(40)

        self.servo_label = QtWidgets.QLabel(self.gridLayoutWidget_15)
        self.servo_label.setFont(font)
        self.servo_label.setObjectName("label_28")
        self.tab2_layout8.addWidget(self.servo_label, 0, 0)

        self.battery_label = QtWidgets.QLabel(self.gridLayoutWidget_14)
        self.battery_label.setFont(font)
        self.battery_label.setObjectName("label_29")
        self.tab2_layout7.addWidget(self.battery_label, 0, 0)

        self.mission1encord_label = QtWidgets.QLabel(self.gridLayoutWidget_16)
        self.mission1encord_label.setObjectName("label")
        self.gridLayout_9.addWidget(self.mission1encord_label, 0, 0, 1, 1)

        self.mission2encord_label = QtWidgets.QLabel(self.gridLayoutWidget_17)
        self.mission2encord_label.setObjectName("label_2")
        self.gridLayout_10.addWidget(self.mission2encord_label, 0, 0, 1, 1)

        self.mission1cord_label = QtWidgets.QLabel(self.coordinates_frame)
        self.mission1cord_label.setGeometry(QtCore.QRect(890, 30, 55, 16))
        self.mission1cord_label.setObjectName("label_3")

        self.mission2cord_label = QtWidgets.QLabel(self.coordinates_frame)
        self.mission2cord_label.setGeometry(QtCore.QRect(890, 150, 55, 16))
        self.mission2cord_label.setObjectName("label_4")

        self.example_label = QtWidgets.QLabel(self.coordinates_frame)
        self.example_label.setGeometry(QtCore.QRect(890, 280, 75, 16))
        self.example_label.setObjectName("label_5")



        self.mission1encord_label.setText("ENTER CORDİNATES : ")
        self.mission2encord_label.setText("ENTER CORDİNATES : ")

        self.mission1cord_label.setText("Mission 1")
        self.mission2cord_label.setText("Mission 2")

        self.example_label.setText("EXAMPLE")

        self.uav_localization_label.setText("UAV LOCALIZATION")

        self.longitude_label.setText("Longitude :")
        self.altitude_label.setText("Altitude :")
        self.bar_height_label.setText("Airspeed :")
        self.heading_label.setText("Heading :")
        self.latitude_label.setText("Latitude :")


        self.uav_imudata_label.setText("UAV IMU DATA")
        self.speed_z_label.setText("Speed Z :")
        self.speed_y_label.setText("Speed Y :")
        self.speed_x_label.setText("Speed X :")
        self.pitch_label.setText("Pitch :")
        self.yaw_label.setText("Yaw :")
        self.roll_label.setText("Roll :")


        self.uav_status_label.setText("UAV STATUS")
        self.flying_label.setText("Flying :")
        self.battery_current_label.setText("Battery Current :")
        self.battery_currentstatus_label.setText("0.0")
        self.flying_status_label.setText("-")


        self.mission1_label.setText("Mission 1")
        self.control1_panel_label.setText("Control Panel")
        self.mission1_time_label.setText("Mission Time")


        self.mission2_label.setText("Mission 2")
        self.control2_panel_label.setText("Control Panel")
        self.mission2_time_label.setText("Mission Time")

        self.servo_label.setText("  SERVO")
        self.servo_label.setStyleSheet('color : red')

        self.battery_label.setText("BATTERY")
        self.battery_label.setStyleSheet('color : red')


        self.altitude_label.setStyleSheet('color : blue')
        self.latitude_label.setStyleSheet('color : blue')
        self.longitude_label.setStyleSheet('color : blue')
        self.bar_height_label.setStyleSheet('color : blue')
        self.heading_label.setStyleSheet('color : blue')
        self.speed_z_label.setStyleSheet('color : blue')
        self.speed_y_label.setStyleSheet('color : blue')
        self.speed_x_label.setStyleSheet('color : blue')
        self.yaw_label.setStyleSheet('color : blue')
        self.pitch_label.setStyleSheet('color : blue')
        self.roll_label.setStyleSheet('color : blue')
        self.battery_current_label.setStyleSheet('color : blue')
        self.flying_label.setStyleSheet('color : blue')
        self.battery_currentstatus_label.setStyleSheet('color : yellow')
        self.flying_status_label.setStyleSheet('color : yellow')


    def pushbutton_(self):
        self.mission_follow_pushbutton = QtWidgets.QPushButton(self.page_1)
        font = QtGui.QFont()
        font.setUnderline(True)
        self.mission_follow_pushbutton.setFont(font)
        self.mission_follow_pushbutton.setObjectName("pushButton_10")
        self.mission_follow_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.mission_follow_pushbutton.setGeometry(QtCore.QRect(1270, 455, 93, 28))

        self.close_terminal_pushbutton = QtWidgets.QPushButton(self.page_1)
        font = QtGui.QFont()
        font.setUnderline(True)
        self.close_terminal_pushbutton.setFont(font)
        self.close_terminal_pushbutton.setObjectName("close_terminal")
        self.close_terminal_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.close_terminal_pushbutton.setGeometry(QtCore.QRect(1370, 455, 93, 28))

        self.close_terminal_pushbutton.setText("Close Terminal")

        self.mission1_start_pushbutton = QtWidgets.QPushButton(self.gridLayoutWidget_4)
        font = QtGui.QFont()
        font.setUnderline(True)
        self.mission1_start_pushbutton.setFont(font)
        self.mission1_start_pushbutton.setObjectName("pushButton_10")
        self.mission1_start_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.gridLayout_4.addWidget(self.mission1_start_pushbutton, 2, 1, 1, 1)

        self.mission1_takeoff_pushbutton = QtWidgets.QPushButton(self.gridLayoutWidget_4)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/newPrefix/takeoff-the-plane.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.mission1_takeoff_pushbutton.setIcon(icon)
        self.mission1_takeoff_pushbutton.setText("")
        self.mission1_takeoff_pushbutton.setObjectName("pushButton_9")
        self.mission1_takeoff_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.gridLayout_4.addWidget(self.mission1_takeoff_pushbutton, 2, 0, 1, 1)

        self.mission1_stop_pushbutton = QtWidgets.QPushButton(self.gridLayoutWidget_4)
        font = QtGui.QFont()
        font.setUnderline(True)
        self.mission1_stop_pushbutton.setFont(font)
        self.mission1_stop_pushbutton.setObjectName("pushButton_3")
        self.mission1_stop_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.gridLayout_4.addWidget(self.mission1_stop_pushbutton, 2, 2, 1, 1)

        self.mission1_landing_pushbutton = QtWidgets.QPushButton(self.gridLayoutWidget_4)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/newPrefix/plane-landing.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.mission1_landing_pushbutton.setIcon(icon)
        self.mission1_landing_pushbutton.setText("")
        self.mission1_landing_pushbutton.setObjectName("pushButton_16")
        self.mission1_landing_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.gridLayout_4.addWidget(self.mission1_landing_pushbutton, 2, 3, 1, 1)

        self.mission2_start_pushbutton = QtWidgets.QPushButton(self.gridLayoutWidget_6)
        font = QtGui.QFont()
        font.setUnderline(True)
        self.mission2_start_pushbutton.setFont(font)
        self.mission2_start_pushbutton.setObjectName("pushButton_15")
        self.mission2_start_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.gridLayout_6.addWidget(self.mission2_start_pushbutton, 2, 1, 1, 1)

        self.mission2_takeoff_pushbutton = QtWidgets.QPushButton(self.gridLayoutWidget_6)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/newPrefix/takeoff-the-plane.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.mission2_takeoff_pushbutton.setIcon(icon)
        self.mission2_takeoff_pushbutton.setText("")
        self.mission2_takeoff_pushbutton.setObjectName("pushButton_12")
        self.mission2_takeoff_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.gridLayout_6.addWidget(self.mission2_takeoff_pushbutton, 2, 0, 1, 1)

        self.mission2_stop_pushbutton = QtWidgets.QPushButton(self.gridLayoutWidget_6)
        font = QtGui.QFont()
        font.setUnderline(True)
        self.mission2_stop_pushbutton.setFont(font)
        self.mission2_stop_pushbutton.setObjectName("pushButton_13")
        self.mission2_stop_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.gridLayout_6.addWidget(self.mission2_stop_pushbutton, 2, 2, 1, 1)

        self.mission2_landing_pushbutton = QtWidgets.QPushButton(self.gridLayoutWidget_6)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/newPrefix/plane-landing.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.mission2_landing_pushbutton.setIcon(icon)
        self.mission2_landing_pushbutton.setText("")
        self.mission2_landing_pushbutton.setObjectName("pushButton_14")
        self.mission2_landing_pushbutton.setStyleSheet("background-color : rgb(20, 255, 50)")
        self.gridLayout_6.addWidget(self.mission2_landing_pushbutton, 2, 3, 1, 1)

        self.mission1_show_pushbutton = QtWidgets.QPushButton(self.coordinates_frame)
        self.mission1_show_pushbutton.setObjectName("pushButton_5")
        self.mission1_show_pushbutton.setGeometry(QtCore.QRect(795, 110, 93, 28))

        self.mission1_clear_pushbutton = QtWidgets.QPushButton(self.coordinates_frame)
        self.mission1_clear_pushbutton.setObjectName("pushButton_6")
        self.mission1_clear_pushbutton.setGeometry(QtCore.QRect(930, 110, 93, 28))

        self.mission2_show_pushbutton = QtWidgets.QPushButton(self.coordinates_frame)
        self.mission2_show_pushbutton.setObjectName("pushButton_5")
        self.mission2_show_pushbutton.setGeometry(QtCore.QRect(795, 235, 93, 28))

        self.mission2_clear_pushbutton = QtWidgets.QPushButton(self.coordinates_frame)
        self.mission2_clear_pushbutton.setObjectName("pushButton_6")
        self.mission2_clear_pushbutton.setGeometry(QtCore.QRect(930, 235, 93, 28))

        self.mission_follow_pushbutton.setText("Follow")
        self.mission1_landing_pushbutton.setEnabled(False)
        self.mission1_takeoff_pushbutton.setEnabled(False)
        self.mission1_stop_pushbutton.setEnabled(False)

        self.mission2_landing_pushbutton.setEnabled(False)
        self.mission2_takeoff_pushbutton.setEnabled(False)
        self.mission2_stop_pushbutton.setEnabled(False)

        self.mission1_show_pushbutton.setText("Show")
        self.mission1_clear_pushbutton.setText("Clear")

        self.mission2_show_pushbutton.setText("Show")
        self.mission2_clear_pushbutton.setText("Clear")

        self.mission1_start_pushbutton.setText("START")
        self.mission2_start_pushbutton.setText("START")
        self.mission1_stop_pushbutton.setText("STOP")
        self.mission2_stop_pushbutton.setText("STOP")


    def textbrowser_(self):
        # font = QtGui.QFont()
        # font.setBold(True)
        # font.setPointSize(15)
        # font.setWeight(20)
        
        self.gridLayout_12 = QtWidgets.QGridLayout(self.console_frame)
        self.gridLayout_12.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_12.setObjectName("gridLayout12")
        self.terminal = EmbTerminal()
        self.gridLayout_12.addWidget(self.terminal)

        # self.console_textbrowser = QtWidgets.QTextBrowser(self.page_1)
        # self.palette = QtGui.QPalette()
        # self.palette.setColor(QtGui.QPalette.Text, QtCore.Qt.black)
        # self.console_textbrowser.setGeometry(QtCore.QRect(0, 420, 800, 30))
        # self.console_textbrowser.setPalette(self.palette)

        # self.console_textbrowser.setFont(font)
        # self.console_textbrowser.setObjectName("textBrowser_2")
        # self.console_textbrowser.setStyleSheet('background-color: rgb(255, 255, 255)')

        font = QtGui.QFont()
        font.setPointSize(11)
        font.setWeight(50)
        font.setBold(True)

        self.map_lat_lng_label = QtWidgets.QLabel(self.page_1)
        self.map_lat_lng_label.setGeometry(QtCore.QRect(0, 420, 400, 30))
        self.map_lat_lng_label.setFont(font)
        self.map_lat_lng_label.setWordWrap(False)
        self.map_lat_lng_label.setAlignment(QtCore.Qt.AlignCenter)
        self.map_lat_lng_label.setObjectName("label_7")
        self.map_lat_lng_label.setStyleSheet('color: red')

    def widgets_(self):
        self.adi = qfi_ADI.qfi_ADI(self.widgets_tab)
        self.adi.resize(240, 240)
        self.adi.reinit()
        self.tab2_layout.addWidget(self.adi, 0, 0)

        self.alt = qfi_ALT.qfi_ALT(self.widgets_tab)
        self.alt.resize(240, 240)
        self.alt.reinit()
        self.tab2_layout2.addWidget(self.alt, 0, 0)

        self.hsi = qfi_HSI.qfi_HSI(self.widgets_tab)
        self.hsi.resize(240, 240)
        self.hsi.reinit()
        self.tab2_layout3.addWidget(self.hsi, 0, 0)

        self.si = qfi_SI.qfi_SI(self.widgets_tab)
        self.si.resize(240, 240)
        self.si.reinit()
        self.tab2_layout4.addWidget(self.si, 0, 0)

        self.vsi = qfi_VSI.qfi_VSI(self.widgets_tab)
        self.vsi.resize(240, 240)
        self.vsi.reinit()
        self.tab2_layout5.addWidget(self.vsi, 0, 0)

        self.tc = qfi_TC.qfi_TC(self.widgets_tab)
        self.tc.resize(240, 240)
        self.tc.reinit()
        self.tab2_layout6.addWidget(self.tc, 0, 0)

        self.RoundBar1 = QRoundProgressBar(self.widgets_tab)
        self.RoundBar1.setObjectName("RoundBar1")
        self.RoundBar1.setValue(0.4)
        self.tab2_layout7.addWidget(self.RoundBar1, 1, 0)

        self.RoundBar2 = QRoundProgressBar(self.widgets_tab)
        self.RoundBar2.setValue(0.5)
        self.RoundBar2.setObjectName("RoundBar1")
        self.tab2_layout8.addWidget(self.RoundBar2, 1, 0)

    def radiansToDegrees(self, radians):
        return radians * 180.0 / math.pi

    def degreesToRadians(self, degrees) :
        return degrees * math.pi / 180.0

    def getBearingBetweenTwoPoints1(self,lat_1, lng_1, lat_2, lng_2):              
        lat1 = self.degreesToRadians(lat_1)
        long1 = self.degreesToRadians(lng_1)
        lat2 = self.degreesToRadians(lat_2)
        long2 = self.degreesToRadians(lng_2)

        dLon = (long2 - long1)

        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)

        radiansBearing = math.atan2(y, x)

        return self.radiansToDegrees(radiansBearing)


    def serial_ports(self):                                                                                            
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        self.result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                self.result.append(port)
            except (OSError, serial.SerialException):
                pass

        if self.result == []:
            self.port_actionmain.setText("NO SERİAL PORT")
        else:
            self.serialPort_name  = self.result[0]
            self.port_actionmain.setText(self.serialPort_name)


    def connected_func(self, connected, armed, mode):
        if connected == "T" and not self.arr[0]:
            self.arr[0] = True
            self.arr[1] = False
            playsound("./speech/connectedtrue_mode.mp3")
        elif connected == "F" and not self.arr[1]:
            self.arr[1] = True
            self.arr[0] = False
            playsound("./speech/connectedfalse_mode.mp3")
        self.armed_func(armed)
        self.mode_func(mode)

    def armed_func(self, armed):
        if armed == "T" and not self.arr[2]:
            self.arr[2] = True
            self.arr[3] = False
            playsound("./speech/armed_mode.mp3")
        elif armed == "F" and not self.arr[3]:
            self.arr[3] = True
            self.arr[2] = False
            playsound("./speech/disarmed_mode.mp3")

    
    def mode_func(self, mode):
        if mode == "MANUAL" and not self.arr[4]:
            self.arr[4:] = [False] * 27
            self.arr[4] = True
            playsound("./speech/manual_mode.mp3")
        elif mode == "CIRCLE" and not self.arr[5]:
            self.arr[4:] = [False] * 27
            self.arr[5] = True
            playsound("./speech/circle_mode.mp3")
        elif mode == "STABILIZE" or mode == "STABILIZED"  and not self.arr[6]:
            self.arr[4:] = [False] * 27
            self.arr[6] = True
            playsound("./speech/stabilize_mode.mp3")
        elif mode == "TRAINING" and not self.arr[7]:
            self.arr[4:] = [False] * 27
            self.arr[7] = True
            playsound("./speech/training_mode.mp3")
        elif mode == "ACRO" and not self.arr[8]:
            self.arr[4:] = [False] * 27
            self.arr[8] = True
            playsound("./speech/acro_mode.mp3")
        elif mode == "AUTOTUNE" and not self.arr[9]:
            self.arr[4:] = [False] * 27
            self.arr[9] = True
            playsound("./speech/autotune_mode.mp3")
        elif mode == "AUTO" and not self.arr[10]:
            self.arr[4:] = [False] * 27
            self.arr[10] = True
            playsound("./speech/auto_mode.mp3")
        elif mode == "RTL" and not self.arr[11]:
            self.arr[4:] = [False] * 27
            self.arr[11] = True
            playsound("./speech/rtl_mode.mp3")
        elif mode == "LOITER" and not self.arr[12]:
            self.arr[4:] = [False] * 27
            self.arr[12] = True
            playsound("./speech/loiter_mode.mp3")
        elif mode == "LAND" and not self.arr[13]:
            self.arr[4:] = [False] * 27
            self.arr[13] = True
            playsound("./speech/land_mode.mp3")
        elif mode == "GUIDED" and not self.arr[14]:
            self.arr[4:] = [False] * 27
            self.arr[14] = True
            playsound("./speech/guided_mode.mp3")
        elif mode == "INITIALISING" and not self.arr[15]:
            self.arr[4:] = [False] * 27
            self.arr[15] = True
            playsound("./speech/initialising_mode.mp3")
        elif mode == "QSTABILIZE" and not self.arr[16]:
            self.arr[4:] = [False] * 27
            self.arr[16] = True  
            playsound("./speech/qstabilize_mode.mp3")
        elif mode == "QHOVER" and not self.arr[17]:
            self.arr[4:] = [False] * 27
            self.arr[17] = True
            playsound("./speech/qhover_mode.mp3")
        elif mode == "QLOITER" and not self.arr[18]:
            self.arr[4:] = [False] * 27
            self.arr[18] = True
            playsound("./speech/qloiter_mode.mp3")
        elif mode == "QLAND" and not self.arr[19]:
            self.arr[4:] = [False] * 27
            self.arr[19] = True
            playsound("./speech/qland_mode.mp3")
        elif mode == "QRTL" and not self.arr[20]:
            self.arr[4:] = [False] * 27
            self.arr[20] = True
            playsound("./speech/qrtl_mode.mp3")
        elif mode == "ALTCTL" and not self.arr[21]:
            self.arr[4:] = [False] * 27
            self.arr[21] = True
            playsound("./speech/altctl_mode.mp3")
        elif mode == "POSCTL" and not self.arr[22]:
            self.arr[4:] = [False] * 27
            self.arr[22] = True
            playsound("./speech/posctl_mode.mp3")
        elif mode == "OFFBOARD" and not self.arr[23]:
            self.arr[4:] = [False] * 27
            self.arr[23] = True
            playsound("./speech/offboard_mode.mp3")
        elif mode == "RATTITUDE" and not self.arr[24]:
            self.arr[4:] = [False] * 27
            self.arr[24] = True
            playsound("./speech/rattitude_mode.mp3")
        elif mode == "AUTO.MISSION" and not self.arr[25]:
            self.arr[4:] = [False] * 27
            self.arr[25] = True
            playsound("./speech/automission_mode.mp3")
        elif mode == "AUTO.LOITER" and not self.arr[26]:
            self.arr[4:] = [False] * 27
            self.arr[26] = True
            playsound("./speech/autoloiter_mode.mp3")
        elif mode == "AUTO.RTL" and not self.arr[27]:
            self.arr[4:] = [False] * 27
            self.arr[27] = True
            playsound("./speech/autortl_mode.mp3")
        elif mode == "AUTO.LAND" and not self.arr[28]:
            self.arr[4:] = [False] * 27
            self.arr[28] = True
            playsound("./speech/autoland_mode.mp3")
        elif mode == "AUTO.RTGS" and not self.arr[29]:
            self.arr[4:] = [False] * 27
            self.arr[29] = True
            playsound("./speech/autortgs_mode.mp3")
        elif mode == "AUTO.READY" and not self.arr[30]:
            self.arr[4:] = [False] * 27
            self.arr[30] = True
            playsound("./speech/autoready_mode.mp3")
        elif mode == "AUTO.TAKEOFF" and not self.arr[31]:
            self.arr[4:] = [False] * 27
            self.arr[31] = True
            playsound("./speech/autotakeoff_mode.mp3")

    def thread_map_func(self, lat , lng):
        try:
            self.longitude_lcdNumber.display(float(lng))
            self.latitude_lcdNumber.display(float(lat))

            if self.latLng_flag == True:
                self.mission_marker = L.marker([float(lat), float(lng)])
                self.map.addLayer(self.mission_marker)
                if self.follow_flag == True:
                    self.map.setView([float(lat), float(lng)], 19)
                self.coordinates_array.append([float(lat), float(lng)])
                self.latLng_flag = False
            else:
                self.coordinates_array.append([float(lat), float(lng)])
                number_of_array = len(self.coordinates_array)
                lat_long = self.coordinates_array[number_of_array - 2:]
                lat_1 = lat_long[0][0]
                lng_1 = lat_long[0][1]
                lat_2 = lat_long[1][0]
                lng_2 = lat_long[1][1]
                self.degrees = self.getBearingBetweenTwoPoints1(lat_1, lng_1, lat_2, lng_2) - 90
                if len(self.coordinates_array) > 20:
                    self.coordinates_array = self.coordinates_array[len(self.coordinates_array) - 15:]
                    self.map.removeLayer(self.mission_polyline)
                    print(len(self.coordinates_array))

                self.mission_polyline = L.polyline(self.coordinates_array, options="{color: 'red'}")
                self.mission_marker.setLatLng([float(lat), float(lng)])
                self.mission_marker.runJavaScript(
                    self.mission_marker.jsName + ".setRotationAngle({});".format(self.degrees))
                if self.follow_flag == True:
                    self.map.setView([float(lat), float(lng)], 19)
                self.map.addLayer(self.mission_polyline)
        except:
            pass
    
    def survey_grid_verify(self):
        if int(self.lineEdit_SurveyGrid_dist.text()) > int(self.lineEdit_SurveyGrid_rad.text()):
            self.info_m = QMessageBox.information(self, "Survey Grid Verify", "Okey")
            return True
        else:
            self.critical_m = QMessageBox.critical(self, "Survey Grid Verify", "Distance must bigger than turn radius")
            return False

    def port_read(self):
        self.latLng_flag = True
        self.circle_flag = True
        self.coordinates_array = []
        if hasattr(self,'mission1_cordi'):                                                             
            self.mission1_cordi = eval(self.mission1_cordi)
            numberof_coordinates = [[0, len(self.mission1_cordi)]]
            data = numberof_coordinates + self.mission1_cordi
            #self.r.rpush('outgoing_messages', "{}".format(data))
            
        elif hasattr(self, 'mission2_cordi'):
            self.mission2_cordi = eval(self.mission2_cordi)
            numberof_coordinates = [[0, len(self.mission2_cordi)]]
            data = numberof_coordinates + self.mission2_cordi
            #self.r.rpush('outgoing_messages', "{}".format(data))

        try:
            vehicle = connect('/dev/ttyUSB0', baud=57600)
        except:
            print("BOOM")
            vehicle.close()
            exit()

        time.sleep(5)

        while True:                                                                                                                                                                                          
            if self.mission2_flag == True or self.mission1_flag == True:  
                time.sleep(1)
                #incoming_msg = self.r.rpop('incoming_messages')
                try:
                    lat = str(vehicle.location.global_frame.lat)
                    lng = str(vehicle.location.global_frame.lon)

                    t_map = Thread(target=self.thread_map_func, args=(lat, lng))
                    t_map.start()

                    connected = str(vehicle.system_status.state)
                    armed = str(vehicle.armed)
                    mode = str(vehicle.mode.name)
                    x = str(vehicle.velocity[0])
                    y = str(vehicle.velocity[1])
                    z = str(vehicle.velocity[2])
                    alt = str(vehicle.location.global_frame.alt)
                    bat = str(vehicle.battery.voltage)
                    roll = str(vehicle.attitude.roll)
                    pitch = str(vehicle.attitude.pitch)
                    yaw = str(vehicle.attitude.yaw)
                    heading = str(vehicle.heading)
                    airspeed = str(vehicle.airspeed)

                    #self.flying_status_label.setText(vehicle.last_heartbeat)

                    #t3 = Thread(target= self.connected_func, args=(connected, armed, mode))
                    #t3.start()

                    self.altitude_lcdNumber.display(float(alt))
                    self.speedx_lcdNumber.display(float(x))
                    self.speedy_lcdNumber.display(float(y))
                    self.speedz_lcdNumber.display(float(z))
                    self.yaw_lcdNumber.display(float(yaw))
                    self.roll_lcdNumber.display(float(roll))
                    self.pitch_lcdNumber.display(float(pitch))
                    self.heading_lcdNumber.display(float(heading))
                    self.barheight_lcdNumber.display(float(airspeed))

                    if float(bat) > -0.4:
                        self.battery_currentstatus_label.setText("good")
                    else:
                        self.battery_currentstatus_label.setText("bad")


                    self.y.append(float(alt))
                    self.y_2.append(float(x))
                    self.y_3.append(float(y))
                    self.y_4.append(float(z))
                    self.y_5.append(float(bat))

                    self.x.append(self.x[-1] + 1)

                    self.data_line.setData(self.x, self.y)
                    self.data_line_2.setData(self.x, self.y_2)
                    self.data_line_3.setData(self.x, self.y_3)
                    self.data_line_4.setData(self.x, self.y_4)
                    self.data_line_5.setData(self.x, self.y_5)

                    lenght = len(self.y)

                    if lenght > 20:
                        self.y = self.y[lenght-15:]  # Remove the first
                        self.y_2 = self.y_2[lenght-15:]  # Remove the first
                        self.y_3 = self.y_3[lenght-15:]  # Remove the first
                        self.y_4 = self.y_4[lenght-15:]  # Remove the first
                        self.y_5 = self.y_5[lenght-15:]  # Remove the first
                        self.x = self.x[lenght-15:]

                    self.adi.setRoll(100*float(roll))
                    self.adi.setPitch(100*float(pitch))

                    self.alt.setAltitude(float(alt)*100)

                    self.hsi.setHeading(float(heading))

                    self.RoundBar1.setValue(float(bat))
                    self.RoundBar2.setValue(0.7)


                    if "-" in x:
                        self.si.setSpeed(float(x) * -1)
                    else:
                        self.si.setSpeed(float(x))

                    self.tc.setTurnRate(10*float(roll))
                    #self.tc.setSlipSkid(10*math.cos(float(yaw)))

                    self.vsi.setClimbRate(float(z)*100)

                    self.vsi.viewUpdate.emit()
                    self.adi.viewUpdate.emit()
                    self.alt.viewUpdate.emit()
                    self.hsi.viewUpdate.emit()
                    self.si.viewUpdate.emit()
                    self.tc.viewUpdate.emit()

                    # if len(incoming_msg_sep) > 20 and self.circle_flag:
                    #     print("Circle")
                    #     circle_lat = incoming_msg_sep[20]
                    #     circle_lon = incoming_msg_sep[22]
                    #     mission_circle = L.circle([circle_lat, circle_lon], 10)
                    #     self.map.addLayer(mission_circle)
                    #     self.circle_flag = False
                except:
                    pass
            else:
                print("DONE -------> Restart")
                vehicle.close()
                break

    def connect_(self):
        self.home_actionmain.triggered.connect(self.on_home)
        self.coordinates_actionmain.triggered.connect(self.on_cordinates)
        self.port_actionmain.triggered.connect(self.on_serialport)

        self.overlap_calculate_pushbutton.clicked.connect(self.on_overlap_calculate)
        self.survey_verify_pushbutton.clicked.connect(self.on_verify)

        self.mission_follow_pushbutton.clicked.connect(self.on_follow)
        self.close_terminal_pushbutton.clicked.connect(self.on_close_terminal)
        self.mission1_start_pushbutton.clicked.connect(self.on_click_mission1start)
        self.mission1_stop_pushbutton.clicked.connect(self.on_click_mission1stop)
        self.mission1_takeoff_pushbutton.clicked.connect(self.on_click_mission1take)
        self.mission1_landing_pushbutton.clicked.connect(self.on_click_mission1land)                                            
        self.mission2_start_pushbutton.clicked.connect(self.on_click_mission2start)
        self.mission2_stop_pushbutton.clicked.connect(self.on_click_mission2stop)
        self.mission2_takeoff_pushbutton.clicked.connect(self.on_click_mission2take)
        self.mission2_landing_pushbutton.clicked.connect(self.on_click_mission2land)

        self.readMission_pushbutton.clicked.connect(self.on_click_readmission)
        self.writeMission_pushbutton.clicked.connect(self.on_click_writemission)
        
        self.moveup_pushbutton.clicked.connect(self.on_click_moveup)
        self.movedown_pushbutton.clicked.connect(self.on_click_movedown)
        self.delete_pushbutton.clicked.connect(self.on_click_delete)
        self.clearMission_pushbutton.clicked.connect(self.on_click_clearmission)


        self.mission1_show_pushbutton.clicked.connect(self.on_mission1_show)
        self.mission1_clear_pushbutton.clicked.connect(self.on_mission1_clear)
        self.mission2_show_pushbutton.clicked.connect(self.on_mission2_show)
        self.mission2_clear_pushbutton.clicked.connect(self.on_mission2_clear)

        self.buttonBox.button(QtWidgets.QDialogButtonBox.Save).clicked.connect(self.on_mission1_cord)
        self.buttonBox.button(QtWidgets.QDialogButtonBox.Cancel).clicked.connect(self.on_mission1_cord_delete)
        self.buttonBox_2.button(QtWidgets.QDialogButtonBox.Save).clicked.connect(self.on_mission2_cord)
        self.buttonBox_2.button(QtWidgets.QDialogButtonBox.Cancel).clicked.connect(self.on_mission2_cord_delete)
        self.buttonBox_createPlan.button(QtWidgets.QDialogButtonBox.Cancel).clicked.connect(self.on_cplan_cancel)
        self.buttonBox_createPlan.button(QtWidgets.QDialogButtonBox.Save).clicked.connect(self.on_cplan_save)
        self.buttonBox_createPlan_2.button(QtWidgets.QDialogButtonBox.Cancel).clicked.connect(self.on_cplan_cancel_2)
        self.buttonBox_createPlan_2.button(QtWidgets.QDialogButtonBox.Save).clicked.connect(self.on_cplan_save_2)
        self.buttonBox_createmarker.button(QtWidgets.QDialogButtonBox.Cancel).clicked.connect(self.on_cmarker_cancel)
        self.buttonBox_createmarker.button(QtWidgets.QDialogButtonBox.Save).clicked.connect(self.on_cmarker_save)
        
        self.buttonBox_load.button(QtWidgets.QDialogButtonBox.Save).clicked.connect(self.on_loadMission)
        
        self.buttonBox_SurveyGrid_lat_lng.button(QtWidgets.QDialogButtonBox.Cancel).clicked.connect(self.on_survey_lat_lng_cancel)
        self.buttonBox_SurveyGrid_lat_lng.button(QtWidgets.QDialogButtonBox.Save).clicked.connect(self.on_survey_lat_lng_save)

        self.buttonBox_SurveyGrid_list.button(QtWidgets.QDialogButtonBox.Cancel).clicked.connect(self.on_survey_list_cancel)
        self.buttonBox_SurveyGrid_list.button(QtWidgets.QDialogButtonBox.Save).clicked.connect(self.on_survey_list_save)

    def on_click_clearmission(self):
        self.tableWidget.clearContents()

    def on_click_delete(self):
        self.tableWidget.removeRow(0)
        for i in range(100):
            for k in range(12):
                if self.tableWidget.item(i, k) is not None:
                    if k == 0:
                        self.tableWidget.setItem(i,0, QTableWidgetItem("{}".format(i+1)))

    def on_loadMission(self):
        print("Load Mission")
        try:
            self.vehicle = connect(connection_string, baud=57600)
            self.upload_mission('./missions/ardupilot/' + self.lineEdit_edit_name.text())
            self.save_mission('exportedmission.txt')
            print("Close vehicle object")
            self.vehicle.close()
            #Print exported file (for demo purposes only)
            self.printfile('exportedmission.txt')
        except:
            pass
    
    def on_click_writemission(self):
        print("Write")
        output='QGC WPL 110\n'
        l = []
        for i in range(100):
            if self.tableWidget.item(i, 0) is not None:
                if i == 0:
                    for _ in range(2):
                        for k in range(12):
                            l.append(self.tableWidget.item(i, k).text())
                        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (l[0],l[1],l[2],l[3],l[4],l[5],l[6],l[7],l[8],l[9],l[10],l[11])
                        output+=commandline
                        l = []
                else:
                    for k in range(12):
                        l.append(self.tableWidget.item(i, k).text())
                    commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (l[0],l[1],l[2],l[3],l[4],l[5],l[6],l[7],l[8],l[9],l[10],l[11])
                    output+=commandline
                    l = []
        print(output)
        if len(output) > 20:
            aFileName = self.lineEdit_write_name.text()
            with open('./missions/ardupilot/' + aFileName, 'w') as file_:
                print(" Write mission to file")
                file_.write(output)

    def on_click_readmission(self):
        aFileName = self.lineEdit_edit_name.text()
        with open('./missions/ardupilot/' + aFileName) as f:
            for i, line in enumerate(f):
                if i==0:
                    if not line.startswith('QGC WPL 110'):
                        raise Exception('File is not supported WP version')
                elif i == 1:
                    pass
                else:
                    linearray=line.split('\t')
                    linearray[-1] = linearray[-1].strip()
                    for k, l in enumerate(linearray):
                        self.tableWidget.setItem(i-2,k, QTableWidgetItem(str(l)))
                    
    def on_click_moveup(self):
        self.tableWidget.insertRow(0)
        for i in range(100):
            for k in range(12):
                if self.tableWidget.item(i, k) is not None:
                    if k == 0:
                        self.tableWidget.setItem(i,0, QTableWidgetItem("{}".format(i+1)))

        if self.tableWidget.item(1, 0) is not None:
            self.tableWidget.setItem(0,0, QTableWidgetItem("{}".format(1)))

    def on_click_movedown(self):
        #print(self.tableWidget.takeItem(0, 1))
        print(self.tableWidget.item(0, 1).text())

    def stopwatch1(self, s_time):
        start_time = s_time
        while self.mission1_flag == True:
            time.sleep(1)
            current_time = time.time()
            sec = current_time - start_time                                                                     
            mins = sec // 60
            sec = sec % 60
            hours = mins // 60
            mins = mins % 60
            self.mission1time_lcdNumber.display("{0}:{1}:{2}".format(int(hours),int(mins),int(sec)))
        pass

    def stopwatch2(self, s_time):
        start_time = s_time
        while self.mission2_flag == True:
            time.sleep(1)
            current_time = time.time()
            sec = current_time - start_time
            mins = sec // 60
            sec = sec % 60
            hours = mins // 60
            mins = mins % 60
            self.mission2time_lcdNumber.display("{0}:{1}:{2}".format(int(hours),int(mins),int(sec)))
        pass


    def edit_waypoint_list(self, waypoint_list):
        lat = []
        lng = [] 
        lat_list = []
        lng_list = []

        lat_lng_list = []

        b = waypoint_list.split("\n")

        for i in range(len(b)):
            c = b[i].split(":")	
            lat_list.append(c[1].split(","))
            lng_list.append(c[2].split("}"))
            lat.append(float(lat_list[i][0].split(" ")[1]))
            lng.append(float(lng_list[i][0].split(" ")[1]))
            lat_lng_list.append([lat[-1], lng[-1]])

        return lat_lng_list
    
    def implement_survey_grid(self, wp_list, distance, angle, alt):
        surveygrid = SurveyGrid.SurveyGrid()
        surveygrid.get_angle(angle)
        surveygrid.get_dist_corridros(distance)
        surveygrid.get_coords(wp_list)
        surveygrid.get_alt(alt)
        #surveygrid.battery_req(int(self.lineEdit_SurveyGrid_vel.text()))

        mission_coordinates = surveygrid.get_wps_latlon()

        dist = 0
        mission_list = [0]
        i = 0
        counter = 0
        self.circle_list = []

        while i < len(mission_coordinates) - 1:
            dist += geopy.distance.distance(mission_coordinates[i], mission_coordinates[i + 1]).m
            flight_time = (dist / int(self.lineEdit_SurveyGrid_vel.text())) / 60
            
            if flight_time > float(self.lineEdit_SurveyGrid_time.text()):
                mission_list.append(i)
                dist = 0
                print(flight_time)
                print(dist)
                counter += 1

                for k in range(mission_list[-2], mission_list[-1]):
                    if k == mission_list[-2]:
                        self.circle_list.append(L.circle([mission_coordinates[k][0], mission_coordinates[k][1]], 7, options="{color: 'white'}"))
                        self.map.addLayer(self.circle_list[k])

                    elif k == mission_list[-1] - 1:
                        self.circle_list.append(L.circle([mission_coordinates[k+1][0], mission_coordinates[k+1][1]], 9, options="{color: 'black'}"))
                        self.map.addLayer(self.circle_list[k])
                    else:
                        self.circle_list.append(L.circle([mission_coordinates[k][0], mission_coordinates[k][1]], 5))
                        self.map.addLayer(self.circle_list[k])
    
                #BUG

                self.mission_polyline_survey = L.polyline(mission_coordinates[mission_list[-2]:mission_list[-1]+1], options="{color: '%s'}" % self.colors[counter])
                self.map.addLayer(self.mission_polyline_survey)
                print(counter, len(mission_coordinates[mission_list[-2]:mission_list[-1]+1]), mission_coordinates[mission_list[-2]:mission_list[-1]+1])
                #mission_polygon = L.polyline(mission_coordinates, options="{color : 'green'}")
                #self.map.addLayer(mission_polygon)
                surveygrid.save_to_txt(self.lineEdit_SurveyGrid_name.text()+str(counter), mission_coordinates[mission_list[-2]:mission_list[-1]+1])

            i += 1

        counter += 1
        mission_list.append(len(mission_coordinates))

        for k in range(mission_list[-2], mission_list[-1]):
            if k == mission_list[-2]:
                self.circle_list.append(L.circle([mission_coordinates[k][0], mission_coordinates[k][1]], 7, options="{color: 'white'}"))
                self.map.addLayer(self.circle_list[k])
            elif k == mission_list[-1] - 1:
                print("Last")
                self.circle_list.append(L.circle([mission_coordinates[k][0], mission_coordinates[k][1]], 9, options="{color: 'black'}"))
                self.map.addLayer(self.circle_list[k])
            else:
                print("else",k)
                self.circle_list.append(L.circle([mission_coordinates[k][0], mission_coordinates[k][1]], 5))
                self.map.addLayer(self.circle_list[k])

        print(len(self.circle_list))

        #BUG
        self.mission_polyline_survey = L.polyline(mission_coordinates[mission_list[-2]:mission_list[-1]], options="{color: '%s'}" % self.colors[counter])
        self.map.addLayer(self.mission_polyline_survey)
        surveygrid.save_to_txt(self.lineEdit_SurveyGrid_name.text()+str(counter), mission_coordinates[mission_list[-2]:mission_list[-1]+1])



    def create_mission(self, miss):
        dosya = open("./missions/px4/{}".format(self.lineEdit_createPlan_name.text()), "w")

        text_init_file = """#!/usr/bin/env python

#####################################
#Created by TEAM IMU GROUND STATION #
#-----umutdumandag61@gmail.com------#
#####################################


import rospy
import mavros
from geometry_msgs.msg import Point, PoseStamped, Pose, TwistStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import *
from mavros import command

#Author: Bugra Buyukarslan

class Modes:
    def __init__(self):
        pass

    def setArm(self):
        print("\\n----------armingCall----------")
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def auto_set_mode(self):
        print("\\n----------autoMode----------")
        rospy.wait_for_service('mavros/set_mode')
        try:
            setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="AUTO.MISSION")
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e

    def wpPush(self,index,wps):
        print("\\n----------pushingWaypoints----------")
        rospy.wait_for_service('mavros/mission/push')
        try:
            wpPushService = rospy.ServiceProxy('mavros/mission/push', WaypointPush,persistent=True)
            wpPushService(start_index=0,waypoints=wps)		# start_index = the index at which we want the mission to start
            print "Waypoint Pushed"
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e
    
    def wpPull(self,wps):
        print("\\n----------pullingWaypoints----------")
        rospy.wait_for_service('mavros/mission/pull')
        try:
            wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull,persistent=True)
            print wpPullService().wp_received

            print "Waypoint Pulled"
        except rospy.ServiceException, e:
            print "Service Puling call failed: %s"%e


class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        
    def stateCb(self, msg):
        self.state = msg

class wpMissionCnt:

    def __init__(self):
        self.wp =Waypoint()
        
    def setWaypoints(self,frame,command,is_current,autocontinue,param1,param2,param3,param4,x_lat,y_long,z_alt):
        self.wp.frame =frame 
        self.wp.command = command
        self.wp.is_current= is_current
        self.wp.autocontinue = autocontinue  
        self.wp.param1=param1 
        self.wp.param2=param2
        self.wp.param3=param3
        self.wp.param4=param4
        self.wp.x_lat= x_lat 
        self.wp.y_long=y_long
        self.wp.z_alt= z_alt 

        return self.wp


def main():

    rospy.init_node('waypointMission', anonymous=True)
    stateMt = stateMoniter()

    md = Modes()

    rate = rospy.Rate(20.0)
    md.setArm()
    rate.sleep()
    md.auto_set_mode()
    rate.sleep()

        """

        dosya.write(text_init_file)

        text_miss_cnt = """
    wayp{} = wpMissionCnt()
        """

        text_miss_waypoint = """
    w = wayp{}.setWaypoints(3,{},False,True,0.0,0.0,0.0,float('nan'),{},{},{})
    wps.append(w)
        """

        text_miss_waypoint_t = """
    w = wayp{}.setWaypoints(3,{},True,True,0.0,0.0,0.0,float('nan'),{},{},{})
    wps.append(w)
        """

        lat = []
        lng = [] 
        lat_list = []
        lng_list = []
        command = []
        alt = []

        b = miss.split("\n")


        for i in range(len(b)):
            c = b[i].split(":")	
            lat_list.append(c[1].split(","))
            lng_list.append(c[2].split("}"))
            lat.append(lat_list[i][0].split(" ")[1])
            lng.append(lng_list[i][0].split(" ")[1])
            command.append(c[3])
            alt.append(c[4])

        #print(lat)
        #print(lng)
        #print(command)
        #print(alt)

        for i in range(len(b)):
            dosya.write(text_miss_cnt.format(i))

        text_mid_file = """
    wps = [] #List to story waypoints
        """

        dosya.write(text_mid_file)

        for i in range(len(b)):
            if i != 0:
                dosya.write(text_miss_waypoint.format(i, command[i], lat[i], lng[i], alt[i]))
            else:
                dosya.write(text_miss_waypoint_t.format(i, command[i], lat[i], lng[i], alt[i]))


        text_end_file = """



    print wps
    md.wpPush(0,wps)
    md.wpPull(0)

    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


        """

        dosya.write(text_end_file)
        dosya.close()

    @pyqtSlot()
    
    def on_verify(self):
        self.survey_grid_verify()
        
    def on_overlap_calculate(self):
        try:
            distance = float(self.lineEdit_overlap_1.text())
            focal_lenght = float(self.lineEdit_overlap_2.text())
            altitude = float(self.lineEdit_overlap_3.text())
            widht = float(self.lineEdit_overlap_4.text())

            percentage = (1 - ((distance * focal_lenght) / (altitude * widht))) * 100

            self.lineEdit_overlap_5.setText(str(round(percentage, 2)))
        except:
            self.critical_m = QMessageBox.critical(self, "Overlap Calculator Message", "Enter all params")

    def on_follow(self):
        self.number_follow += 1
        if self.number_follow % 2 != 0:
            self.follow_flag = False
            self.mission_follow_pushbutton.setStyleSheet("background-color : rgb(255, 0, 0)")
        else:
            self.follow_flag = True
            self.mission_follow_pushbutton.setStyleSheet("background-color : rgb(0, 255, 0)")

    def on_close_terminal(self):
        self.close_terminal_pushbutton.setStyleSheet("background-color : rgb(255, 0, 0)")
        self.terminal.stop()

    def on_cmarker_save(self):
        self.marker = L.marker([self.lineEdit_marker_lat.text(), self.lineEdit_marker_lon.text()])
        self.map.addLayer(self.marker)

    def on_cmarker_cancel(self):
        self.lineEdit_marker_lat.setText(" ")
        self.lineEdit_marker_lon.setText(" ")
        #     self.map.removeLayer(self.marker)
        
    def on_cplan_save_2(self):
        #print("aaa")
        self.waypoint_list = self.plan_textbrowser_2.toPlainText()
        self.create_mission(self.waypoint_list)
        #print(self.waypoint_list)
        #print(type(self.waypoint_list))

    def on_cplan_cancel_2(self):
        self.plan_textbrowser_2.clear()

    def on_cplan_cancel(self):
        self.lineEdit_createPlan.setText(" ")
        self.lineEdit_createPlan_command.setText(" ")
        self.lineEdit_createPlan_alt.setText("")

    def on_cplan_save(self):
        latlng = self.lineEdit_createPlan.text()
        command = self.lineEdit_createPlan_command.text()
        alt = self.lineEdit_createPlan_alt.text()
        self.plan_textbrowser_2.append("{}:{}:{}".format(latlng, command, alt))



    def on_survey_lat_lng_cancel(self):
        self.lineEdit_SurveyGrid_lat_lng.setText(" ")

    def on_survey_lat_lng_save(self):
        latlng = self.lineEdit_SurveyGrid_lat_lng.text()
        self.SurveyGrid_list_textbrowser.append("{}".format(latlng))


    def on_survey_list_save(self):
        if int(self.lineEdit_SurveyGrid_dist.text()) > int(self.lineEdit_SurveyGrid_rad.text()):   
            self.waypoint_list_survey = self.SurveyGrid_list_textbrowser.toPlainText()
            #print(self.waypoint_list_survey)
            edited_list = self.edit_waypoint_list(self.waypoint_list_survey)
            angle = int(self.lineEdit_SurveyGrid_angle.text())
            distance = int(self.lineEdit_SurveyGrid_dist.text())
            alt = int(self.lineEdit_SurveyGrid_alt.text())
            print(alt)
            self.implement_survey_grid(edited_list, distance, angle, alt)
        else:
            self.critical_m = QMessageBox.critical(self, "Survey Grid Verify", "Distance must bigger than turn radius")

    def on_survey_list_cancel(self):
        self.SurveyGrid_list_textbrowser.clear()
        for i in range(len(self.circle_list)):
            self.map.removeLayer(self.circle_list[i])
        self.map.removeLayer(self.mission_polyline_survey)

    def on_home(self):
        self.stackedWidget.setCurrentIndex(0)

    def on_cordinates(self):
        self.stackedWidget.setCurrentIndex(1)

    def on_serialport(self):
        self.serial_ports()
        if self.result == []:                                                                               
            self.critical_m = QMessageBox.critical(self, "PyQt5 Message", "Please Check Connection")
        else:
            self.information_m = QMessageBox.information(self,"PyQt5 Message","The port you connected to is {}".format(self.serialPort_name))

    def on_mission1_cord(self):
        if hasattr(self,'mission2_cordi'):
            self.critical_m_4 = QMessageBox.critical(self, "PyQt5 Message", "Please enter only one mission")
        else:
            self.mission1_cordi = self.lineEdit.text()
            if self.mission1_cordi == "":
                self.critical_m_2 = QMessageBox.critical(self, "PyQt5 Message", "Please Enter Coordinates")
                pass
            else:
                #self.console_textbrowser.append('{} | Mission 1 saved'.format(time.asctime()))
                self.mission1_polygon = L.polyline(self.mission1_cordi, options="{color : 'green'}")

    def on_mission1_cord_delete(self):
        self.lineEdit.clear()

    def on_mission2_cord(self):
        if hasattr(self,'mission1_cordi'):
            self.critical_m_5 = QMessageBox.critical(self, "PyQt5 Message", "Please enter only one mission")
        else:
            self.mission2_cordi = self.lineEdit_2.text()
            if self.mission2_cordi  == "":
                self.critical_m_3 = QMessageBox.critical(self, "PyQt5 Message", "Please Enter Coordinates")
                pass
            else:
                #self.console_textbrowser.append('{} | Mission 2 saved'.format(time.asctime()))
                self.mission2_polygon = L.polyline(self.mission2_cordi, options="{color: 'green'}")

    def on_mission2_cord_delete(self):
        self.lineEdit_2.clear()

    def on_click_mission1start(self):
            if hasattr(self, "mission1_polygon") == False :                                                     
                QMessageBox.critical(self, 'Pyqt5 message', "Please enter coordinates")
            else:
                self.mission1_start_flag = True
                #self.console_textbrowser.append('{} | Mission 1 is starting'.format(time.asctime()))
                self.mission1_status_frame.setStyleSheet('background : rgb(0, 255, 0)')
                self.flying_status_label.setText('STARTED')
                self.map.addLayer(self.mission1_polygon)
                start_time = time.time()
                self.mission2_flag = False
                self.mission1_flag = True

                self.t1 = Thread(target = self.stopwatch1, args=[start_time])                                   
                self.t2 = Thread(target= self.port_read)                                                        
                self.t1.start()
                self.t2.start()

                self.mission1_landing_pushbutton.setEnabled(True)
                self.mission1_takeoff_pushbutton.setEnabled(True)
                self.mission1_stop_pushbutton.setEnabled(True)

                self.mission2_start_pushbutton.setEnabled(False)
                self.mission2_landing_pushbutton.setEnabled(False)
                self.mission2_takeoff_pushbutton.setEnabled(False)
                self.mission2_stop_pushbutton.setEnabled(False)


    def on_click_mission1stop(self):
        if hasattr(self,"mission1_polygon") == False:
            QMessageBox.critical(self, 'PyQt5 message', "Please start the mission first")
        else:
            #self.console_textbrowser.append('{} | Mission 1 is stopping'.format(time.asctime()))
            self.mission1_status_frame.setStyleSheet('background : rgb(255, 0 , 0)')

            self.flying_status_label.setText('Stop')
            self.mission1_flag = False
            self.map.removeLayer(self.mission1_polygon)

            #self.graphics_()                                                                
            self.mission1_landing_pushbutton.setEnabled(False)
            self.mission1_takeoff_pushbutton.setEnabled(False)
            self.mission1_stop_pushbutton.setEnabled(False)


    def on_click_mission1take(self):
        if hasattr(self,"mission1_polygon") == False:
            QMessageBox.critical(self, 'PyQt5 message', "Please start the mission first")
        else:
            #self.console_textbrowser.append('{} | UAV is leaving'.format(time.asctime()))
            self.mission1_status_frame.setStyleSheet('background : rgb(0, 255, 0)')

    def on_click_mission1land(self):
        if hasattr(self,"mission1_polygon") == False:
            QMessageBox.critical(self, 'PyQt5 message', "Please start the mission first")
        else:
            #self.console_textbrowser.append('{} | UAV is landing'.format(time.asctime()))
            self.mission1_status_frame.setStyleSheet('background : rgb(255, 0 , 0)')

    def on_click_mission2start(self):
            if hasattr(self,"mission2_polygon") == False:
                QMessageBox.critical(self, 'PyQt5 message', "Please enter coordinates")
            else:
                #self.console_textbrowser.append('{} | Mission 2 is starting'.format(time.asctime()))
                self.mission2_status_frame.setStyleSheet('background : rgb(0, 255, 0)')
                self.flying_status_label.setText('Started')
                self.map.addLayer(self.mission2_polygon)
                start_time = time.time()
                self.mission1_flag = False
                self.mission2_flag = True
                self.t1 = Thread(target = self.stopwatch2, args=[start_time])
                self.t2 = Thread(target= self.port_read)
                self.t1.start()
                self.t2.start()
                self.mission2_landing_pushbutton.setEnabled(True)
                self.mission2_takeoff_pushbutton.setEnabled(True)
                self.mission2_stop_pushbutton.setEnabled(True)

                self.mission1_start_pushbutton.setEnabled(False)
                self.mission1_landing_pushbutton.setEnabled(False)
                self.mission1_takeoff_pushbutton.setEnabled(False)
                self.mission1_stop_pushbutton.setEnabled(False)

    def on_click_mission2stop(self):
        if hasattr(self,"mission2_polygon") == False:
            QMessageBox.critical(self, 'PyQt5 message', "Please start the mission first")
        else:
            #self.console_textbrowser.append('{} | Mission 2 is stopping'.format(time.asctime()))
            self.mission2_status_frame.setStyleSheet('background : rgb(255, 0 , 0)')

            self.flying_status_label.setText('STOP')
            self.mission2_flag = False
            self.map.removeLayer(self.mission2_polygon)
            self.graphics_()

            self.mission2_landing_pushbutton.setEnabled(False)
            self.mission2_takeoff_pushbutton.setEnabled(False)
            self.mission2_stop_pushbutton.setEnabled(False)

    def on_click_mission2take(self):
        if hasattr(self,"mission2_polygon") == False:
            QMessageBox.critical(self, 'PyQt5 message', "Please start the mission first")
        else:
            #self.console_textbrowser.append('{} | UAV is leaving'.format(time.asctime()))
            self.mission2_status_frame.setStyleSheet('background : rgb(0, 255, 0)')

    def on_click_mission2land(self):
        if hasattr(self,"mission2_polygon") == False:
            QMessageBox.critical(self, 'PyQt5 message', "Please start the mission first")
        else:
            #self.console_textbrowser.append('{} | UAV is landing'.format(time.asctime()))
            self.mission2_status_frame.setStyleSheet('background : rgb(255, 0 , 0)')

    def on_mission1_show(self):
        self.mission1_show_polygon = L.polygon(self.lineEdit.text())
        self.map.addLayer(self.mission1_show_polygon)

    def on_mission1_clear(self):
        self.map.removeLayer(self.mission1_show_polygon)

    def on_mission2_show(self):
        self.mission2_show_polygon = L.polygon(self.lineEdit_2.text())
        self.map.addLayer(self.mission2_show_polygon)

    def on_mission2_clear(self):
        self.map.removeLayer(self.mission2_show_polygon)


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.showMaximized()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
