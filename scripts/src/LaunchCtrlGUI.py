'''
Created on Aug 9, 2010

@author: mvs
'''

from LaunchCtrl import LaunchCtrl, ArgumentException
from LaunchCtrlProcess import LaunchCtrlSleep
import threading, thread
from PyQt4 import QtGui, QtCore, uic 
import sys
import os
from collections import deque
        
class LaunchCtrlGUI():
    
    def __init__(self, UI_FILE):
        self._filechooserpath = os.getcwd() + "/../../param/"
        self.stopped = True
        self.laststop = False
        self.redIcon = QtGui.QIcon('img/RedLight.png')
        self.greenIcon = QtGui.QIcon('img/GreenLight.png')
        self.sleepIcon = QtGui.QIcon('img/Sleep.png')
        self.gui = uic.loadUi(UI_FILE)
        self.gui.setWindowIcon(QtGui.QIcon('img/JuniorIcon.png'))
        QtCore.QObject.connect(self.gui.startButton, QtCore.SIGNAL('clicked()'), self.start)
        QtCore.QObject.connect(self.gui.stopButton, QtCore.SIGNAL('clicked()'), self.stop)
        QtCore.QObject.connect(self.gui.actionOpen, QtCore.SIGNAL('triggered()'), self.loadFile)
        QtCore.QObject.connect(self.gui.actionExit, QtCore.SIGNAL('triggered()'), self.shutdown)
        QtCore.QObject.connect(self.gui.processList, QtCore.SIGNAL('itemActivated(QListWidgetItem *)'), self.toggle)
        self.gui.actionSave.setEnabled(False)
        self.gui.actionSave_as.setEnabled(False)
        #QtCore.QObject.connect(self.gui.)

        self.pinfo = list()
        self.launcher = None
        
        self.commandoutput = deque(maxlen=10)
        self.outputindex = -1
        
        self.timer = QtCore.QTimer(self.gui)
        QtCore.QObject.connect(self.timer, QtCore.SIGNAL('timeout()'), self.update)
        self.timer.start(100)
        self.gui.show()
   
    def load(self, filename):
        
        self.kill()
        if not filename:
            return
        self.gui.processList.clear()
        self.launcher = LaunchCtrl()
        try:
            self.launcher.load(filename)
        except ArgumentException as e:
            print e.str
            self.kill()
            self.gui.processList.clear()
            del self.pinfo[:]
            return
            
        self.pinfo = list()
        
        for process in self.launcher.processList:
            if isinstance(process, LaunchCtrlSleep):
                self.gui.processList.addItem(QtGui.QListWidgetItem(self.sleepIcon, "".join(("Sleep ", str(process.time)))))
                self.gui.processList.item(self.gui.processList.count() - 1).setFlags(QtCore.Qt.ItemIsSelectable)
                self.pinfo.append("Sleep")
            else:
                self.gui.processList.addItem(QtGui.QListWidgetItem(self.redIcon, process.command()))
                self.pinfo.append(process)
        filename = filename.split('/')
        filename = filename[len(filename) - 1]
        self.gui.setWindowTitle("Junior - " + filename)
    
    def toggle(self, item):
        index = self.gui.processList.row(item)
        process = self.pinfo[index]
        if isinstance(process, str):
            return
        if process.running():
            process.stop()
        else:
            process.start()

        
    def start(self):
        if self.launcher:
            thread.start_new_thread(self.launcher.start, ())

    def stop(self):
        if not self.stopped and self.launcher:
            thread.start_new_thread(self.launcher.stop, ())
        
    def kill(self):
        if self.launcher:
            thread.start_new_thread(self.launcher.kill, ())
        
    def shutdown(self):
        self.laststop = True
        self.kill()
        
    def loadFile(self):
        filename = QtGui.QFileDialog.getOpenFileName(self.gui, "Select LuanchCtrl file...", self._filechooserpath, "LaunchCtrl files (*.ctrl)")
        self.load(filename)
            
    def update(self):
        i = 0
        self.stopped = True
        selected = self.gui.processList.selectedItems()
        if len(selected) == 1:
            selIndex = self.gui.processList.row(selected[0])
            if selIndex != self.outputindex:
                self.outputindex = selIndex
                self.commandoutput.clear()
            if self.pinfo[selIndex].running() and self.pinfo[selIndex]._output:
                for j in range(30):
                    try:
                        new = self.pinfo[selIndex].pty().readline()
                    except IOError:
                        break
                    if new == "":
                        break
                    if j == 29:
                        self.pinfo[selIndex].pty().read()
                    self.commandoutput.append(new)
                data = ""
                for line in self.commandoutput:
                    data += line
                self.gui.processOutput.setText(data)
        for process in self.pinfo:
            if isinstance(process, str):
                i += 1
                continue;
            font = self.gui.processList.item(i).font()
            processtext = process.command()
            if process.running():
                self.gui.processList.item(i).setIcon(self.greenIcon)
                if process.startCount() > 0:  
                    processtext += "  (Starts: %d)" % process.startCount()
                if process.stopping():
                    processtext += " - Killing"
                self.stopped = False
                if process.pty():
                    try:
                        process.pty().read()
                    except IOError:
                        pass
            else:
                self.gui.processList.item(i).setIcon(self.redIcon)
            if processtext != self.gui.processList.item(i).text():
                font.setBold(True)
            else:
                font.setBold(False)
            self.gui.processList.item(i).setFont(font)
            self.gui.processList.item(i).setText(processtext)
            i += 1
        self.timer.start(100)
        
        # Wait for children to exit and then close the window
        if self.laststop:
            if threading.activeCount() == 1:
                self.gui.close()
                
        if self.stopped:
            self.gui.startButton.setEnabled(True)
            self.gui.stopButton.setEnabled(False)
        else:
            self.gui.startButton.setEnabled(False)
            self.gui.stopButton.setEnabled(True)
        
if __name__ == "__main__":
    
    UI_FILE = 'ui/RunUI.ui'
    app = QtGui.QApplication(sys.argv)
    gui = LaunchCtrlGUI(UI_FILE)
    #app.setQuitOnLastWindowClosed(False);

    app.connect(app, QtCore.SIGNAL('lastWindowClosed()'), gui.shutdown)
    #QtCore.QObject.connect(gui.gui, QtCore.SIGNAL('clicked()'), app, QtCore.SLOT('close()'))
    #gui.avc_init()
    sys.exit(app.exec_())
    


