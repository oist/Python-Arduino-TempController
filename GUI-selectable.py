import sys
from PyQt4 import QtCore, QtGui
from ui import Ui_MainWindow
import numpy as np
import pyqtgraph as pg
import serial
from lxml import etree as ET
import datetime

from pyqtgraph.Point import Point
#import xml.etree.ElementTree as ET
#all turn off everything when exit
#add cursor to pyqtgraph done on 20151014
#add fuzzy logc like completed on 20151018 not yet fully fuzzy logic
#add standalone mode in arduino
#add checkbox to select PID  completed on 20151020
#add change of setpoint iwhile still on completed on 20151019
#add saving plot and saving the temperature and time in csv at the end of the experiment.
#draw a horizontal line for the setpoint completed on 20151021
#segment setpoint line on 2015/11/3
#add automatic scaling in time series into hours or minutes
#modify arduino to show state of MAX state complet 20151028
#add in future support of scripts for fully automation



class MyForm(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(MyForm, self).__init__(parent)
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        QtCore.QObject.connect(self.ui.actionAbout, QtCore.SIGNAL('triggered()'), self.about)
        QtCore.QObject.connect(self.ui.connectbut, QtCore.SIGNAL('clicked()'), self.connectarduino)
        QtCore.QObject.connect(self.ui.load_but, QtCore.SIGNAL('clicked()'), self.loadsetting)
        QtCore.QObject.connect(self.ui.save_but, QtCore.SIGNAL('clicked()'), self.savesetting)
        QtCore.QObject.connect(self.ui.radio_MAX_direct, QtCore.SIGNAL('checked()'), self.directctrlchecked)
        self.ui.radio_MAX_direct.toggled.connect(self.directctrlchecked)

        #QtCore.QObject.connect(self.ui.sp_MAX, QtCore.SIGNAL('valueChanged()'), self.maxtoggle)
        self.ui.ctrl_MAX.toggled.connect(self.maxtoggle)
        self.ui.autotune_MAX.toggled.connect(self.maxautotunetoggle)

        #connect change of spinbox value to update the setpoint
        self.ui.sp_MAX.setKeyboardTracking(False)
        self.ui.sp_MAX.valueChanged.connect(self.updatesp)
        self.sp_MAX=25
        self.max_state=False
        self.directctrl=True
        self.setpointcheck=False
        # add a control state to differentiate between direct control and pid state

        #to add a control state showing the heating on the arduino has started.


        self.ser=None #declare arduino
        #create a pyqtgraph for plotting
        self.win=pg.GraphicsWindow()
        self.win.setWindowTitle('live plot from serial')
        self.p = self.win.addPlot()
        self.p.addLegend()
        self.p.setTitle('Python-Arduino Temp Controller')
        self.p.setLabel('bottom', text='Time', units='s')
        self.p.setLabel('left', text='Temp', units='oC')
        self.p.setLabel('top',text="<span style='font-size:12pt'>Time=, <span style='color: red'>Temp=</span>")
        #self.label=pg.LabelItem(justify='right')
        #self.win.addItem(self.label)
        #self.label.setText("<span style='font-size:12pt'>test")

        self.vb=self.p.vb
        self.curve_max = self.p.plot(pen='r', name='MAX31855')
        #self.curve_setpoint = self.p.plot(pen='y', name='Setpoint')
        #try to make setpoint something appendable.
        self.curve_setpoint = []
        self.curve_setpoint.append(self.p.plot(pen='y', name='Setpoint'))
        #problem:setpoint should be able to break


        #adding of crosshair from pyqtgraph examples
        self.vLine = pg.InfiniteLine(pen=0.4, angle=90, movable=False)
        self.hLine = pg.InfiniteLine(pen=0.4, angle=0, movable=False)
        self.p.addItem(self.vLine, ignoreBounds=True)
        self.p.addItem(self.hLine, ignoreBounds=True)


        self.p.scene().sigMouseMoved.connect(self.mouseMoved)
        #proxy = pg.SignalProxy(self.p.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)
        #QtCore.QObject.connect(self.win.scene(), QtCore.SIGNAL('mouseMoved()'), self.mouseMoved)


        #self.curve_mlx = self.p.plot(pen='b', name='MLX90614')
        #to do add a line for setpoint
        self.data_max =[]
        self.data_time = []
        self.setpoint=[]
        self.setpointtime=[]

        self.ydata_time=None
        self.ydata_max=None
        self.ysetpoint=None
        self.ysetpointtime=None

        #declaration for discontinuous setpoint curve
        self.array_index = 0

        #delaration of timer for update graph
        self.timer = QtCore.QTimer(self)
        QtCore.QObject.connect(self.timer, QtCore.SIGNAL('timeout()'), self.updategraph)
        #declare factor of ctrl cmd
        self.ctrlsys = 'a'
        self.ctrlstate = '0'
        self.ctrltune = '0'
        self.ctrlsetpoint = 0.00
        self.ctrlkp = 0.00
        self.ctrlti= 0.00
        self.ctrltd = 0.00
        self.ctrlcmd = "0"
        #declare factors for autotune
        self.autotunetimer = QtCore.QTimer(self)
        QtCore.QObject.connect(self.autotunetimer, QtCore.SIGNAL('timeout()'), self.autotuneupdate)
        self.autotunestart = False
        self.autotunestate = False
        self.t0temp=0
        self.t0=0
        self.t1temp=0
        self.t1=0
        self.t2temp=0
        self.t2=0
        #declare for xml
        self.xml_root=None
        self.loadsetting()

    @QtCore.pyqtSlot(bool)
    def directctrlchecked(self, checked):
        if not checked:
            self.ui.MAX_kc.setEnabled(True)
            self.ui.MAX_ti.setEnabled(True)
            self.ui.MAX_td.setEnabled(True)
            self.ui.autotune_MAX.setEnabled(True)
            self.directctrl=False
        elif checked:
            self.ui.MAX_kc.setEnabled(False)
            self.ui.MAX_ti.setEnabled(False)
            self.ui.MAX_td.setEnabled(False)
            self.ui.autotune_MAX.setEnabled(False)
            self.directctrl=True


    def updatesp(self):
        self.sp_MAX = self.ui.sp_MAX.value()
        if self.max_state is True:
            try:
                self.ser.open()
            except:
                #print "already open"
                pass
            #self.ctrlcmd = '1,%s' % (str(self.ui.sp_MAX.value()))
            self.ctrlcmd = '1,%s' % (str(self.sp_MAX))
            #print self.ctrlcmd
            #self.ctrlcmd = 'a,1,0,%s,%s,%s,%s' % (self.ui.sp_MAX.value(), self.ui.MAX_kc.value(), self.ui.MAX_ti.value(), self.ui.MAX_td.value())
            self.ser.write(self.ctrlcmd.encode())
            print "Setpoint:%s at %s. (%s)" % (str(self.sp_MAX),str(self.ydata_time[-1]), datetime.datetime.now().strftime('%Y.%m.%d.%H:%M:%S'))

    def mouseMoved(self, pos):
        try:
            mousePoint=self.vb.mapSceneToView(pos)
            testtime = (np.abs(self.ydata_time-mousePoint.x())).argmin()
            self.vLine.setPos(self.ydata_time[testtime])
            self.hLine.setPos(self.ydata_max[testtime])
            #self.label.setText("<span style='font-size:12pt'>Time=%0.1f, <span style='color: red'>Temp=%0.1f</span>" % (self.ydata_time[testtime], self.ydata_max[testtime]))
            self.p.setLabel('top', text="<span style='font-size:12pt'>Time=%0.1f, <span style='color: red'>Temp=%0.1f</span>" % (self.ydata_time[testtime], self.ydata_max[testtime]))
        except:
            pass

            #print self.ydata_time[testtime]
            #print self.ydata_max[testtime]


        #print "%s,%s" % (mousePoint.x(), mousePoint.y())
        '''
        index = int(mousePoint.x())
        if index >0 and index<len(self.data_max):
            self.label.setText("<span style='font-size: 12pt'>x=%0.1f,   <span style='color: red'>y1=%0.1f</span>" % (index, self.data_max[index]))
            self.vLine.setPos(mousePoint.x())
            self.hLine.setPos(mousePoint.y())
        #self.vLine.setPos(self.win.pos.x())
'''
        '''
        pos = []
        mousePoint = pg.mapScenetoView(pos)
        self.vLine.setPos(mousePoint.x())
        self.hLine.setPos(mousePoint.y())
        #pos = evt[0]  ## using signal proxy turns original arguments into a tuple
        '''
        '''
        if self.p.sceneBoundingRect().contains(pos):
            mousePoint = self.vb.mapSceneToView(pos)
            index = int(mousePoint.x())
            if index > 0 and index < len(self.data_max):

            vLine.setPos(mousePoint.x())
            hLine.setPos(mousePoint.y())
            '''



    def loadsetting(self):
        #load setting
        #set sepoint, kp, ti, td, calibrator
        tree = ET.parse('config.xml')
        xml_root = tree.getroot()
        #self.ui.comlist.clear()
        #self.ui.comlist.addItem(xml_root.find('Port').text.strip())
        self.ui.comlisttext.setPlainText(xml_root.find('Port').text.strip())
        self.ui.MAX_calib.setValue(float(xml_root.find('MAX_Calibration').text.strip()))
        self.ui.sp_MAX.setValue(float(xml_root.find('MAX_Setpoint').text.strip()))
        self.ui.MAX_kc.setValue(float(xml_root.find('MAX_Kc').text.strip()))
        self.ui.MAX_ti.setValue(float(xml_root.find('MAX_Ti').text.strip()))
        self.ui.MAX_td.setValue(float(xml_root.find('MAX_Td').text.strip()))
        print "load setting dated on %s" % xml_root.find('date').text.strip()

    def savesetting(self):
        declare = ET.Element('Config')
        currdate = ET.SubElement(declare, 'date')
        i=datetime.datetime.now()
        currdate.text = i.strftime('%Y.%m.%d')
        comlist = ET.SubElement(declare, 'Port')
        #comlist.text = "test"
        #comlist.text = str(self.ui.comlist.currentText())
        comlist.text=str(self.ui.comlisttext.toPlainText())
        maxcalib = ET.SubElement(declare, 'MAX_Calibration')
        maxcalib.text = str(self.ui.MAX_calib.value())
        maxsp = ET.SubElement(declare, 'MAX_Setpoint')
        maxsp.text = str(self.ui.sp_MAX.value())
        maxkc = ET.SubElement(declare, 'MAX_Kc')
        maxkc.text = str(self.ui.MAX_kc.value())
        maxti = ET.SubElement(declare, 'MAX_Ti')
        maxti.text = str(self.ui.MAX_ti.value())
        maxtd = ET.SubElement(declare, 'MAX_Td')
        maxtd.text = str(self.ui.MAX_td.value())
        tree = ET.ElementTree(declare)
        tree.write('config.xml', pretty_print=True, xml_declaration=True)
        print "setting saved"


    @QtCore.pyqtSlot(bool)
    def maxtoggle(self, checked):
        if not checked:
            #send turn off on max pid to arduino
            try:
                self.ser.open()
            except:
                pass
            self.ctrlcmd = '0'
            self.ser.write(self.ctrlcmd.encode())
            self.max_state=False
            print "Heater turned off at %s.(%s)" % (str(self.ydata_time[-1]), datetime.datetime.now().strftime('%Y.%m.%d %H:%M:%S'))
            palette= QtGui.QPalette(self.ui.ctrl_MAX.palette())
            palette.setColor(QtGui.QPalette.ButtonText, QtGui.QColor('black'))
            self.ui.ctrl_MAX.setPalette(palette)
            #a=self.data_time[-1]
            #self.setpointtime.append(a)
            #self.setpoint.append(20)
            self.setpointcheck=True

           #self.timer.stop()
            #self.cleargraph()
            #print "turn off MAx"
        elif checked:
            #send turn on to max pid to arduino
            try:
                self.ser.open()
            except:
                #print "already open"
                pass
            # add if control to send data between direct
            #self.ctrlcmd = '1,%s' % (str(self.ui.sp_MAX.value()))
            if self.directctrl is True:
                self.ctrlcmd = '1,%s' % (str(self.sp_MAX))
            else:
                self.ctrlcmd = '2,%s,%s,%s,%s' % (str(self.sp_MAX),str(self.ui.MAX_kc.value()), str(self.ui.MAX_ti.value()), str(self.ui.MAX_td.value()))
            #print self.ctrlcmd
            #self.ctrlcmd = 'a,1,0,%s,%s,%s,%s' % (self.ui.sp_MAX.value(), self.ui.MAX_kc.value(), self.ui.MAX_ti.value(), self.ui.MAX_td.value())
            self.ser.write(self.ctrlcmd.encode())
            #start new plotting
            #self.timer.stop()
            #self.cleargraph()
            self.max_state=True
            self.timer.start(100)
            #add array_index for plotting next setpoint line
            self.array_index += 1
            self.curve_setpoint.append(self.p.plot(pen='y'))
            self.setpointtime=[]
            self.setpoint=[]
            #self.ysetpointtime = np.array([])
            #self.ysetpoint = np.array([])
            print "Heater turned on at %s. (%s)" % (str(self.ydata_time[-1]),datetime.datetime.now().strftime('%Y.%m.%d %H:%M:%S'))
            palette= QtGui.QPalette(self.ui.ctrl_MAX.palette())
            palette.setColor(QtGui.QPalette.ButtonText, QtGui.QColor('red'))
            self.ui.ctrl_MAX.setPalette(palette)
            if self.setpointcheck is True: # detect if setpoint has been changed
                #a=self.data_time[-1]
                #self.setpointtime.append(a)
                #self.setpoint.append(20)
                self.setpointcheck=False
            #print "turn on MAX"


    @QtCore.pyqtSlot(bool)
    def maxautotunetoggle(self, checked):
        if not checked:
            try:
                self.ser.open()
            except:
                pass
            #self.calculatepid(sys="MAX")
            #self.ui.MAX_kc.setValue(self.ctrlkp)
            #self.ui.MAX_ti.setValue(self.ctrlti)
            #self.ui.MAX_td.setValue(self.ctrltd)
            self.ctrlcmd = '0'
            self.ser.write(self.ctrlcmd.encode())
            self.autotunestate=False
            self.autotunetimer.stop()
            #print "complete autotune on max"
        elif checked:
            try:
                self.ser.open()
            except:
                pass
            self.autotunestate = True
            self.autotunestart = True
            self.ctrlcmd = '1,100'
            self.ser.write(self.ctrlcmd.encode())

            # wait 3 seconds until take t0.
            #self.t0temp= self.data_max[-1]
            #self.t0=float(self.data_time[-1]) #in seconds
            #print "t0 temp is"
            #print self.t0temp
            #print "t0 is"
            #print self.t0
            #print "start autotune on max"

    def calculatepid(self, **options):
        if options.get("sys") == "MAX":
            t1tempaim = self.t0temp*1.05 # make t1 temp at time as more than 5% increase in temperature
            t1tempindex = [n for n, i in enumerate(self.data_max) if i>t1tempaim][0]
            self.t1temp = self.data_max[t1tempindex]
            self.t1 = self.data_time[t1tempindex]

            #t2tempaim = self.t0temp*1.1
            #t2tempindex = [n for n, i in enumerate(self.data_max) if i>t2tempaim][0]
            #if (t2tempindex==t1tempindex):
            #    t2tempindex+=1
            #self.t2temp = self.data_max[t2tempindex]
            #self.t2 = self.data_time[t2tempindex]
            # change t2 temp and t2 as long the tuning process is true
            self.t2temp = self.data_max[-1]
            self.t2 = self.data_time[-1]
            m = (self.t2temp-self.t1temp)/(self.t2-self.t1)
            #XXXX need to modify this part
            normslope = m/50
            deadtime = (self.t0temp - (self.t2temp/(m*self.t2)))/m
            #print m
            #print normslope
            #print deadtime
            self.ctrlkp=1.2/(deadtime*normslope)
            self.ctrlti = 2.0*deadtime
            self.ctrltd= 0.5*deadtime
            #print self.ctrlkp
            #print self.ctrlti
            #print self.ctrltd

        elif options.get("sys") == "DS":
            t1tempaim = self.t0temp*1.05
            t1tempindex = [n for n, i in enumerate(self.data_ds) if i>t1tempaim][0]
            self.t1temp = self.data_ds[t1tempindex]
            self.t1 = self.data_time[t1tempindex]
            t2tempaim = self.t0temp*1.1
            t2tempindex = [n for n, i in enumerate(self.data_ds) if i>t2tempaim][0]
            if (t2tempindex==t1tempindex):
                t2tempindex+=1
            self.t2temp = self.data_ds[t2tempindex]
            self.t2 = self.data_time[t2tempindex]
            #print "calculate for DS"
            m = (self.t2temp-self.t1temp)/(self.t2-self.t1)
            normslope = m/50
            deadtime = (self.t0temp - (self.t2temp/(m*self.t2)))/m
            #print m
            #print normslope
            #print deadtime
            self.ctrlkp=1.2/(deadtime*normslope)
            self.ctrlti = 2.0*deadtime
            self.ctrltd=0.5*deadtime
            #print self.ctrlkp
            #print self.ctrlti
            #print self.ctrltd

        else:
            print "error:unknown target system"

    @QtCore.pyqtSlot(str)
    def on_Log_message(self, message): #display console txt into the ui's textedit
        Console = self.ui.console
        Console.moveCursor(QtGui.QTextCursor.End)
        Console.insertPlainText(message)

    def about(self):
        QtGui.QMessageBox.__init__(self)
        QtGui.QMessageBox.about(self, 'About', """Python Temperature Controller through Arduino I/O <p> Developed by Hsieh-Fu Tsai (hsiehfutsai@radiagnostic.com)
         <p> This script initiates temperature control based on MAX31855 K type thermocouple, DS18B20 one-wire temperature sensor, and MLX90614 IR MEMS thermopile sensor.""")

    def connectarduino(self):
        try:
            port = str(self.ui.comlisttext.toPlainText())
            self.ser = serial.Serial(port, 9600)
            self.ui.connectbut.setText("Connected")
            self.ui.connectbut.setEnabled(False)
            self.ui.groupBox.setEnabled(True)
            self.timer.start(300)
            line = self.ser.readline().rstrip().split(",")
        except:
            print ('failed')

    def cleargraph(self):
        #self.ser.close()
        self.data_time=[]
        self.data_max=[]
        self.setpoint =[]

    def autotuneupdate(self):
        #update autotune factors
        self.calculatepid(sys="MAX")
        self.ui.MAX_kc.setValue(self.ctrlkp)
        self.ui.MAX_ti.setValue(self.ctrlti)
        self.ui.MAX_td.setValue(self.ctrltd)

    def updategraph(self):
        try:
            self.ser.open()
        except:
            #print "already open"
            pass
        #global curve_max, curve_dx, curve_mlx, data_max, data_ds, data_mlx, curve_time, data_time
        line = self.ser.readline().rstrip().split(",")
        #print line
        #update label in the gui
        self.ui.MAX_raw.setText(line[2])
        maxtemp = float(line[2])+self.ui.MAX_calib.value()
        self.ui.MAX_Temp.setText(str(maxtemp))
        self.data_time.append(float(line[0])/1000.0)
        self.data_max.append(maxtemp)
        #show only setpoint graph if during heating.
        if self.max_state is True:
            self.setpointtime.append(float(line[0])/1000.0)
            self.setpoint.append(self.ui.sp_MAX.value())

        #self.data_mlx.append(float(line[7]))
        self.ydata_time = np.array(self.data_time, dtype='float64')
        self.ydata_max = np.array(self.data_max, dtype='float64')
        self.ysetpoint=np.array(self.setpoint, dtype='float64')
        self.ysetpointtime=np.array(self.setpointtime, dtype='float64')

        #ydata_mlx = np.array(self.data_mlx, dtype='float64')
        self.curve_max.setData(x=self.ydata_time, y=self.ydata_max)
        self.curve_setpoint[self.array_index].setData(x=self.ysetpointtime, y=self.ysetpoint)
        #self.curve_mlx.setData(x=ydata_time, y=ydata_mlx)
        #print line[1]
        if (self.autotunestart is True and line[1] == '1'):
            self.t0temp = self.data_max[-1]
            self.t0 = float(self.data_time[-1]) #in seconds
            #print self.t0temp
            #print self.t0
            self.autotunestart = False
            self.autotunetimer.start(100)
        else:
            pass
            #find the first time point that arduino starts to heat as t0


class Log(QtCore.QObject):
    message = QtCore.pyqtSignal(str)
    def __init__(self, parent=None):
        super(Log, self).__init__(parent)
    def write(self, message):
        self.message.emit(str(message))


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    myapp = MyForm()
    myapp.show()
    Log = Log()
    Log.message.connect(myapp.on_Log_message)
    sys.stdout=Log
    sys.exit(app.exec_())

