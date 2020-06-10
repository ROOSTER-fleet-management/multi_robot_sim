#!/usr/bin/env python

import rospy
import roslaunch
import rospkg
from robot_class import Robot
import sys
from PyQt4 import QtGui, QtCore

print("Test04")

class Window(QtGui.QMainWindow):

    def __init__(self):
        super(Window, self).__init__()
        self.setGeometry(50, 50, 500, 300)
        self.setWindowTitle("PyQT tuts!")
        # self.setWindowIcon(QtGui.QIcon('pythonlogo.png'))

        #region Main Menu (The menu at the top of the application)
        # File - Save file
        saveFile = QtGui.QAction("&Save file", self)
        saveFile.setShortcut("Ctrl+S")
        saveFile.setStatusTip("Save file")
        saveFile.triggered.connect(self.save_file)

        # File - Open file
        openFile = QtGui.QAction("&Open file", self)
        openFile.setShortcut("Ctrl+O")
        openFile.setStatusTip("Open file")
        openFile.triggered.connect(self.open_file)

        # File - Quit application
        extractActionQuit = QtGui.QAction("&Quit application", self)
        extractActionQuit.setShortcut("Ctrl+Q")
        extractActionQuit.setStatusTip('Leave the app')
        extractActionQuit.triggered.connect(self.close_application)

        # Test - This is a test menu action
        extractActionTest = QtGui.QAction("&Test this action", self)
        extractActionTest.setShortcut("Ctrl+T")
        extractActionTest.setStatusTip("Testing the menu")
        extractActionTest.triggered.connect(self.custom_method)

        # Test - This is a test menu action, the second one.
        extractActionTest2 = QtGui.QAction("&Also test this action", self)
        extractActionTest2.setShortcut("Ctrl+Shift+T")
        extractActionTest2.setStatusTip("Testing the menu and shortcuts")
        extractActionTest2.triggered.connect(self.custom_method)

        # Editor - Basic text editor
        extractActionEditor = QtGui.QAction("&Editor", self)
        extractActionEditor.setShortcut("Ctrl+E")
        extractActionEditor.setStatusTip("Open basic text Editor")
        extractActionEditor.triggered.connect(self.editor)

        # Add a status bar at the button which can help with explaining items in the gui or show messages/actions.
        self.statusBar()

        # Add the main menu bar itself, populate with menu items.
        mainMenu = self.menuBar()
        fileMenu = mainMenu.addMenu('&File')
        testMenu = mainMenu.addMenu('&Test')
        editorMenu = mainMenu.addMenu('&Editor')
        fileMenu.addAction(saveFile)
        fileMenu.addAction(openFile)
        fileMenu.addAction(extractActionQuit)
        testMenu.addAction(extractActionTest)
        testMenu.addAction(extractActionTest2)
        editorMenu.addAction(extractActionEditor)
        #endregion

        # Home screen
        self.home()

    def home(self):
        btn = QtGui.QPushButton("Click me! I'm a button :)", self)
        # btn.clicked.connect(QtCore.QCoreApplication.instance().quit)
        btn.clicked.connect(self.custom_method)
        btn.resize(btn.minimumSizeHint())
        btn.move(10, 100)


        #region Adding Home toolbar
        r = rospkg.RosPack()
        savefile_image = r.get_path("multi_robot_sim")+'/scripts/ui/images/Save.png' 
        extractAction = QtGui.QAction(QtGui.QIcon(savefile_image), 'Save current setup', self)
        extractAction.triggered.connect(self.custom_method)

        colorAction = QtGui.QAction('Color', self)
        colorAction.triggered.connect(self.color_picker)

        self.toolBar = self.addToolBar("Main Toolbar")
        self.toolBar.addAction(extractAction)
        self.toolBar.addAction(colorAction)
        #endregion

        #region Adding a checkbox
        checkBox = QtGui.QCheckBox("Enlarge Window", self)
        checkBox.move(10, 60)
        checkBox.resize(checkBox.minimumSizeHint())
        checkBox.stateChanged.connect(self.enlarge_window)
        #endregion

        #region Progress bar
        self.progress = QtGui.QProgressBar(self)
        self.progress.setGeometry(200, 80, 250, 20)

        self.btn = QtGui.QPushButton("Launch", self)
        self.btn.move(200, 120)
        self.btn.clicked.connect(self.launch_progress)
        #endregion

        #region Drop down button and conncted label
        self.label = QtGui.QLabel("Label text test123", self)

        comboBox = QtGui.QComboBox(self)
        comboBox.addItem("Option 1")
        comboBox.addItem("Option Two")
        comboBox.addItem("Option Thr33")
        comboBox.addItem("Option Four")
        comboBox.addItem("Fifth option")
        comboBox.addItem("Six yo")

        comboBox.move(50, 250)
        comboBox.resize(comboBox.minimumSizeHint())
        self.label.move(50, 150)
        self.label.resize(self.label.sizeHint())
        comboBox.activated[str].connect(self.set_label_text)
        #endregion

        #region Adding a list widget with list items
        robotList = QtGui.QListWidget()
        robotList.resize(300, 120)
        robotList.move(300, 200)
        robotList.addItem("Robot Item 1")
        robotList.addItem("Robot Item 2")
        robotList.addItem("Robot Item 3")
        robotList.addItem("Robot Item 4")
        robotList.itemClicked.connect(self.item_clicked)
        #endregion

        self.show()
    def item_clicked(self, item):
        QtQui.QMessageBox.information(self, "Robot list", "You clicked: "+item.text())
        if item.text() == "Item 4":
            self.addItem("Another!")
        elif item.text() == "Another!":
            removed_item = self.takeItem(self.currentRow())
            del removed_item

    def save_file(self):
        name = QtGui.QFileDialog.getSaveFileName(self, 'Save File')
        file = open(name, 'w')
        text = self.textEdit.toPlainText()
        file.write(text)
        file.close()

    def open_file(self):
        name = QtGui.QFileDialog.getOpenFileName(self, 'Open File')
        file = open(name, 'r')
        self.editor()

        with file:
            text = file.read()
            self.textEdit.setText(text)

    def editor(self):
        self.textEdit = QtGui.QTextEdit()
        self.setCentralWidget(self.textEdit)

    def color_picker(self):
        color = QtGui.QColorDialog.getColor()
        self.label.setStyleSheet("QWidget { background-color: %s}" %color.name())

    def set_label_text(self, text):
        self.label.setText(text)

    def launch_progress(self):
        self.completed = 0

        while self.completed < 100:
            self.completed += 0.0001
            self.progress.setValue(self.completed)


    def enlarge_window(self, state):
        if state == QtCore.Qt.Checked:
            self.setGeometry(50, 50, 1000, 600)
        else:
            self.setGeometry(50, 50, 500, 300)
    
    def custom_method(self):
        print("Custom text printed!")

    def close_application(self):
        choice = QtGui.QMessageBox.question(self, 
                                            'Quit application?',
                                            "Are you sure you want to quit?", 
                                            QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
        
        if choice == QtGui.QMessageBox.Yes:
            print("Closing multi_robot_sim launcher node...")
            QtCore.QCoreApplication.instance().quit()
        else:
            pass
    
    def closeEvent(self, event):
        event.ignore()
        self.close_application()

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('multi_robot_sim_launcher', anonymous=False)
    
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        #print "uuid=", uuid
        
        # Set up launch api objects
        launch_gzb = roslaunch.scriptapi.ROSLaunch()
        launch_rdg = roslaunch.scriptapi.ROSLaunch()

        #region --- GUI ---
        app = QtGui.QApplication(sys.argv)
        GUI = Window()
        app.exec_()
        #endregion

        # Shut down launched nodes
        launch_gzb.parent.shutdown()
        launch_rdg.parent.shutdown()
        print "shut Down sequence complete!"
    except rospy.ROSInterruptException:
        pass