# interfacing dependcies
from qdb_interface import (AsicREG, AsicCMD, AsicEnable, AsicMask,
                           qdb_interface, QDBBadAddr, REG, SAQReg, DEFAULT_PACKET_SIZE)
import os
import sys
import time

# PyQt GUI things
from PyQt5 import QtCore
from PyQt5.QtWidgets import (QWidget, QPushButton, QCheckBox, QSpinBox, QLabel,
                             QDoubleSpinBox, QProgressBar, QTabWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QStatusBar,
                             QDialog, QDialogButtonBox, QLCDNumber, QFileDialog,
                             QSpacerItem, QSizePolicy)
from PyQt5.QtCore import QProcess, QTimer, pyqtSignal
from PyQt5.QtGui import QIcon, QPalette
from PyQt5.QtWidgets import QApplication, QMainWindow, QAction

import pyqtgraph as pg

# for output data
from array import array
import ROOT
import numpy as np
import datetime
# for spawning other process to turn binary data into ROOT
import subprocess 


class QPIX_GUI(QMainWindow):
    close_udp = pyqtSignal()
    
    def __init__(self):
        super(QMainWindow, self).__init__()

        # IO interfaces
        self.qpi = qdb_interface()
        self.close_udp.connect(self.qpi.finish) # closes udp worker thread
        self._tf = ROOT.TFile("./test.root", "RECREATE")
        self._tt = ROOT.TTree("qdbData", "data_tree")
        self._saqMask = 0

        # SAQ meta data information
        self._start_hits = 0
        self._stop_hits = 0
        self.version = self.qpi.version

        # storage tree setup words
        self._data = {
            "trgT" : array('L', [0]),
            "daqT" : array('L', [0]),
            "asicT" : array('L', [0]),
            "asicX" : array('H', [0]),
            "asicY" : array('H', [0]),
            "wordType" : array('H', [0])}
        types = ["trgT/i", "daqT/i", "asicT/i", "asicX/b", "asicY/b", "wordType/b"]
        for data, typ in zip(self._data.items(), types):
            self._tt.Branch(data[0], data[1], typ)

        # window setup
        self.setWindowTitle('SAQ DAQ')

        self.cbChannels = []
        
        # passive triggering
        #self._clock = QTimer()
        #self._clock.timeout.connect(self.trigger)
        #self._lastTrig = -1

        # initialize the sub menus
        self._make_menuBar()

        # create the layouts that are needed for making the GUI pretty
        self.tabW = QTabWidget()
        self.tabW.addTab(self._makeGeneralLayout(), "General")
        self.tabW.addTab(self._makeChannelsLayout(), "Channels")
        self.tabW.addTab(self._makeAdvancedLayout(), "Advanced")
        self.setCentralWidget(self.tabW)

        # show the main window
        self.show()

        # FIXME: may go elsewhere...?
        self.updateChannelMaskOnZybo() 
        
    def _makeChannelsLayout(self):
        self._channelsPage = QWidget()
        layout = QHBoxLayout()
        label = QLabel()
        label.setText("Channels page")
        layout.addWidget(label)
        self._channelsPage.setLayout(layout)
        return self._channelsPage
        
    def _makeAdvancedLayout(self):
        self._advancedPage = QWidget()
        layout = QHBoxLayout()
        label = QLabel()
        label.setText("Advanced page")
        layout.addWidget(label)
        self._advancedPage.setLayout(layout)
        return self._advancedPage
    
    def _makeGeneralLayout(self):
        """
        Wrapper function to make the layout for the main tab for the SAQ DAQ
        into the QMainWindow's QStackedLayout
        """
        self._generalPage = QWidget()
        
        layout = QHBoxLayout()

        ctrlLayout = QHBoxLayout()
        plotLayout = QHBoxLayout()
        layout.addLayout(ctrlLayout)
        layout.addLayout(plotLayout)

        cbLayout = QGridLayout() # channel checkboxes
        btnLayout = QVBoxLayout() # start/stop buttons
        ctrlLayout.addLayout(cbLayout)
        ctrlLayout.addLayout(btnLayout)

        self._generalPage.setLayout(layout)

        # checkboxes
        nrows = 8
        ncols = 2
        for col in range(ncols):
            for row in range(nrows):
                chan = (col*nrows)+row+1
                cb = QCheckBox(f"Ch. {chan}")
                cb.setChecked(True)
                cb.stateChanged.connect(self.updateChannelMaskOnZybo)

                self.cbChannels.append(cb)
                cbLayout.addWidget(cb, row, col, QtCore.Qt.AlignTop)
        vspacer = QSpacerItem(40, 20, QSizePolicy.Minimum, QSizePolicy.Expanding)
        cbLayout.addItem(vspacer, 8, 0, QtCore.Qt.AlignTop)
        
        # start/stop buttons
        btnStart = QPushButton("Start")
        btnStart.clicked.connect(self.startRun)
        btnLayout.addWidget(btnStart)
        
        btnStop = QPushButton("Stop")
        btnStop.clicked.connect(self.stopRun)
        btnLayout.addWidget(btnStop)

        btnClear = QPushButton("Clear Data")
        btnClear.clicked.connect(self.clearData)
        btnLayout.addWidget(btnClear)

        btnSave = QPushButton("Save Data")
        btnSave.clicked.connect(self.saveData)
        btnLayout.addWidget(btnSave)

        btnLayout.addStretch() # vfill at the bottom
        
        graph = pg.PlotWidget()
        #color = self.palette().color(QPalette.Window)
        #graph.setBackground(color)
        graph.setBackground('w')
        graph.setTitle("Graph title")
        graph.setLabel("left", "y-axis label")
        graph.setLabel("bottom", "x-axis label")
        pen1 = pg.mkPen(color=(255,0,0), width=3)
        pen2 = pg.mkPen(color=(0,0,255), width=3)
        line1 = graph.plot([1,2,3],[1,2,3], pen=pen1)
        line2 = graph.plot([1,2,3],[1,4,9], pen=pen2)
        plotLayout.addWidget(graph)

        return self._generalPage


    def startRun(self):
        print("start run clicked")
        self.enableSAQ()

    def stopRun(self):
        print("stop run clicked")

    def clearData(self):
        print("clear data clicked")
        
    def saveData(self):
        print("save data clicked")
        
    ############################
    ## Zybo specific Commands ##
    ############################

    ############################
    ## ASIC specific Commands ##
    ############################

    ############################
    ## SAQ specific Commands  ##
    ############################
    def enableSAQ(self):
        """
        read / write the single bit register at SaqEnable to turn on Axi Fifo streaming.
        then update the saqMask to begin allowing triggers from saq pins
        """
        addr = REG.SAQ(SAQReg.SAQ_ENABLE)
        # restart the thread if we haven't started it yet
        if not self.qpi.thread.isRunning():
            print("restarting udp collection thread")
            self.qpi.thread.start()
        self.qpi.regWrite(addr, 1)

    # FIXME: move to other location in code
    def getChannelMaskFromGUI(self):
        self._saqMask = 0
        for ii, cb in enumerate(self.cbChannels):
            #chan = ii+1
            if cb.isChecked():
                self._saqMask += 1 << ii
            #val = int(cb.isChecked())
            #print(f"chan, val = {chan}, {val}")
        print(f"self._saqMask = {self._saqMask}")
        
    # FIXME: move to other location in code
    def updateChannelMaskOnZybo(self):
        # read the mask value from GUI checkboxes
        self.getChannelMaskFromGUI()
        # send that value to the FPGA
        addr = REG.SAQ(SAQReg.MASK)
        wrote = self.qpi.regWrite(addr, self._saqMask)
        
    def disableSAQ(self):
        """
        read / write the single bit register at SaqEnable to turn off Axi Fifo streaming.
        then update the saqMask to disable triggers from saq pins
        """
        addr = REG.SAQ(SAQReg.SAQ_ENABLE)
        self.qpi.regWrite(addr, 0)
        self._stop_hits = self.getSAQHits()

    def getSAQHits(self):
        """
        read the SAQ Hit register buffer
        """
        addr = REG.SAQ(SAQReg.SAQ_FIFO_HITS)
        hits = self.qpi.regRead(addr)
        return hits

        
    def SaveData(self, output_file=None):
        """
        NOTE: This function is called by default when the GUI closes

        This function handles storing a recorded run at a destination output file.

        This should be the only controlling / call function that uses
        make_root.py to store output data and the metadata tree for SAQ on the
        Zybo.
        """
        if output_file is None:
            output_file = datetime.datetime.now().strftime('./%m_%d_%Y_%H_%M_%S.root')
            print("saving default file", output_file)

        input_file = self.qpi.worker.output_file
        if self.qpi.thread.isRunning():
            self.close_udp.emit()

        found = os.path.isfile(input_file)
        if not found:
            return
        else:
            args = [input_file, output_file, self.version, self._start_hits, self._stop_hits]
            args = [str(arg) for arg in args]
            print(f"args = {args}")
            subprocess.Popen(["python", "make_root.py", *args])

    ###########################
    ## GUI specific Commands ##
    ###########################
    def _make_menuBar(self):
        menubar = self.menuBar()
        menubar.setNativeMenuBar(False)

        # exit action
        exitAct = QAction(QIcon(), '&Exit', self)
        exitAct.setShortcut('Ctrl+Q')
        exitAct.setStatusTip('Exit application')
        exitAct.triggered.connect(self.close)

        # create a way to save the data collected
        saveAct = QAction(QIcon(), '&Save', self)
        saveAct.setShortcut('Ctrl+S')
        saveAct.setStatusTip('Save Data')
        saveAct.triggered.connect(self.SaveAs)

        # add the actions to the menuBar
        fileMenu = menubar.addMenu('File')
        fileMenu.addAction(exitAct)
        fileMenu.addAction(saveAct)

    def closeEvent(self, event):
        print("closing the main gui")
        self.SaveData()

    def SaveAs(self):
        """
        This function is called when the user selects the save option
        from the file under the menu bar.

        a dialog window should appear and the user can select the name
        and location of the output root file to be created.
        """
        fileName = QFileDialog.getSaveFileName(self, "Save Data File",
                                       os.getcwd(),
                                       ".root")

        # don't double up the root extension
        if fileName[0][-5:] == ".root":
            out_file = fileName[0]
        else:
            out_file = fileName[0]+".root"
        self.SaveData(out_file)

if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = QPIX_GUI()
    app.exec_()
