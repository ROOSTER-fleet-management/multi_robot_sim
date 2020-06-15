#!/usr/bin/env python

import rospy
import roslaunch
import rospkg   # for resolving paths to ROS packages 
import math     # for trigonometric evaluations

from docking_station_class import DockingStation
from transformations import Pose2d
from robot_class import Robot
from gazebo_class import Gazebo

import sys
from PyQt4 import QtGui, QtCore
from ui import gui_launcher_node


############################## TODOLIST ##################################
# DONE 1. Add question ' are you sure '  on shutdown.
# DONE 2. Add question prompt ' Are you sure'  on Quit application
# DONE 3. Add about popup information
# DONE 4. Add right mouse button delete items from launch tree
# DONE 5. Add launch tree item adding. 
# DONE 6. Change launch status labeltext and labelicon based on launch_status variable.
# TODO 7. Add clearing everything when clicking the 'New' button in the File menu with question prompt if they are sure.
# WAIT TODO 8. Add saving and loading of docking station variables based on comboBox robot type.
# TODO 9. Add saving and loading of the launch tree
# TODO 10. Combine saving and loading of launch tree and all docking station variables to the File > Save and File > Load menu, allowing to save and load to and from a file.
# DONE 11. Add an icon to the application.
# TODO 12. Add status tips where needed.
# TODO 13. Make a nice icon for the application (256x256 png)
# DONE 14. Add check to make sure clicking launch while already launched won't work. Disable button?
# DONE 15. Disable items in robotTree, add robot button and clear list button when launched.
# TODO 16. Upon launch iterate over the launch list and instantiate robot class objects.
# TODO 17. Implement "multi_robot_sim_launcher.py" functionality inside this script
# TODO 17. a) Add docking station dictionairy
##########################################################################

version = "0.1"

print("Test. Version: "+version)

class launchStatus:             # Kind of like an enum but for python 2.7
    AWAITING = "Awaiting launch..."
    LAUNCHED = "Launched"
    SHUTDOWN = "Shutdown"

launch_status = launchStatus.AWAITING

class GuiMainWindow(gui_launcher_node.Ui_MainWindow, QtGui.QMainWindow):
    def __init__(self):
        super(GuiMainWindow, self).__init__()
        self.setupUi(self)

        self.labelStatusIcon.setPixmap(QtGui.QPixmap(":/icons/How-to.png"))

        # Set up launch_list
        self.launch_list = ["TEST"]

        # Connect launch tab buttons
        self.btnClearLaunchList.clicked.connect(self.clear_launch_tree)
        self.btnLaunch.clicked.connect(self.launch_triggered)
        self.btnShutdown.clicked.connect(self.shutdown_triggered)
        self.robotTree.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.robotTree.customContextMenuRequested.connect(self.open_context_menu)  
        self.btnAddRobot.clicked.connect(self.add_robot)
        self.btnShutdown.setEnabled(False)

        # TreeWidget context menu
        self.treeMenu = QtGui.QMenu('Menu', self)
        deleteItem = QtGui.QAction("&Delete", self)
        deleteItem.setStatusTip("Delete item from Launch list")
        deleteItem.triggered.connect(self.delete_launch_tree_item)
        deleteIcon = QtGui.QIcon()
        deleteIcon.addPixmap(QtGui.QPixmap(":/icons/Close.png"))
        deleteItem.setIcon(deleteIcon)
        self.treeMenu.addAction(deleteItem)

        # Connect docking station tab
        self.labelDockingStationTitle.setStatusTip("testing")

        # File menu
        self.actionNew.setStatusTip("Clear current setup and start with a fresh one. Note: Any unsaved changes will be lost!")
        self.actionSave_file.setStatusTip("Save current launch and docking station setup to a file.")
        self.actionLoad_file.setStatusTip("Open an existing setup file. Note: Any unsaved changes will be lost!")
        self.actionAbout.setStatusTip("Popup window with additional information.")
        self.actionAbout.triggered.connect(self.about)
        self.actionQuit_application.setStatusTip("Quit the application. Note: Any unsaved changes will be lost!")
        self.actionQuit_application.triggered.connect(self.close_application)

    def add_robot(self):
        if launch_status != launchStatus.LAUNCHED:
            # Translate input boxes into treeWidgetItem columns
            robot_type = self.comboBoxRobotType.currentText()
            val = self.spinBoxRobotID.value()
            id_num = ("" if val > 9 else "0") + str(val)
            id_name = ""
            if robot_type == "Ridgeback":
                id_name = "rdg"
            elif robot_type == "Husky":
                id_name = "hsk"
            elif robot_type == "ROSbot":
                id_name = "rsb"
            robot_id = id_name+id_num

            # Iterate over all existing (top level) items (a.k.a. robots) in the robotTree to check if the new Robot ID is unique.
            root = self.robotTree.invisibleRootItem()
            child_count = root.childCount()
            is_unique = True
            for i in range(child_count):
                item = root.child(i)
                # print(item.text(0) + ("New" if item.text(0) != robot_id else "Duplicate")
                if item.text(0) == robot_id:
                    is_unique = False
                    break

            docking_station = self.comboBoxDockingStationLaunch.currentText()
            navigation = "SFM-MPDM" if self.checkBoxSFMMPDM.isChecked() else "move_base"
            # print("Adding robot: " + robot_id + " | " + navigation + " | " + robot_type + " | " + docking_station)
            if is_unique:
                robot_item = QtGui.QTreeWidgetItem([robot_id, navigation, robot_type, docking_station])
                self.robotTree.addTopLevelItem(robot_item)
            else:
                QtGui.QMessageBox.information(self, "Robot ID is not unique.", "The Robot ID you tried to add already exists in the launch list and cannot be added again.")

    def open_context_menu(self):
        self.treeMenu.exec_(QtGui.QCursor.pos())

    def delete_launch_tree_item(self, item):
        if launch_status != launchStatus.LAUNCHED:
            model_index = self.robotTree.currentIndex()
            self.robotTree.takeTopLevelItem(model_index.row())

    def clear_launch_tree(self):
        self.robotTree.clear()
    
    def set_launched_gui(self):
        self.labelStatusText.setText(launch_status)
        self.labelStatusIcon.setPixmap(QtGui.QPixmap(":/icons/Next.png"))
        self.btnLaunch.setEnabled(False)
        self.btnShutdown.setEnabled(True)
        self.btnAddRobot.setEnabled(False)
        self.btnClearLaunchList.setEnabled(False)
        root = self.robotTree.invisibleRootItem()
        child_count = root.childCount()
        is_unique = True
        for i in range(child_count):
            item = root.child(i)
            item.setDisabled(True)
    
    def launch_triggered(self):
        self.launch_list = []    # Empty launch list
        # Iterate over all existing (top level) items (a.k.a. robots) in the robotTree and add as robot class objects
        root = self.robotTree.invisibleRootItem()
        child_count = root.childCount()
        is_unique = True
        for i in range(child_count):
            item = root.child(i)
            robot = Robot(item.text(0))
            # robot.assign_cell(ds01)       # Assign docking station based on item.text(3) as key for docking station dictionairy which has all ds class objects. (ds01, ds02, ds03)
            self.launch_list.append(robot)

        if not self.launch_list:
            # launch_list is empty, throw error.
            QtGui.QMessageBox.warning(self, "Launch list is empty!", "You must have at least 1 robot in the 'Launch list' before you can launch.")
        else:
            global launch_status
            launch_status = launchStatus.LAUNCHED
            print("Launching!")
            print(self.launch_list)
            self.launch_nodes()
            print(launch_status)
            self.set_launched_gui()

    def set_shutdown_gui(self):
        self.labelStatusText.setText(launch_status)
        self.labelStatusIcon.setPixmap(QtGui.QPixmap(":/icons/Abort.png"))
        self.btnLaunch.setEnabled(True)
        self.btnShutdown.setEnabled(False)
        self.btnAddRobot.setEnabled(True)
        self.btnClearLaunchList.setEnabled(True)
        root = self.robotTree.invisibleRootItem()
        child_count = root.childCount()
        is_unique = True
        for i in range(child_count):
            item = root.child(i)
            item.setDisabled(False)

    def shutdown_triggered(self):
        global launch_status
        if launch_status != launchStatus.LAUNCHED:
            # It was not launched yet, thus cannot shutdown.
            QtGui.QMessageBox.information(self, "Nothing to shutdown.", "No launch was found to shutdown.")
        elif QtGui.QMessageBox.Yes == QtGui.QMessageBox.question(self, 
                                            'Shut down launched setup?',
                                            "Are you sure you want to shut down the currently launched setup?", 
                                            QtGui.QMessageBox.Yes | QtGui.QMessageBox.No):
            
            launch_status = launchStatus.SHUTDOWN
            print("Shutting down...")
            self.shutdown_nodes()
            print(launch_status)
            self.set_shutdown_gui()
    
    def close_application(self):
        choice = QtGui.QMessageBox.question(self, 
                                            'Quit application?',
                                            "Are you sure you want to quit? Any unsaved changed will be lost!", 
                                            QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
        
        if choice == QtGui.QMessageBox.Yes:
            print("Closing multi_robot_sim launcher node...")
            QtCore.QCoreApplication.instance().quit()
        else:
            pass
    
    def closeEvent(self, event):
        event.ignore()
        self.close_application()

    def about(self):
        QtGui.QMessageBox.information(self, "About - multi_robot_sim launcher.", "multi_robot_sim launcher. \nVersion: "+version+"\n\nThe ROS package multi_robot_sim is created by the Human Robot Coproduction research group at the Industrial Design Engineering faculty of the Delft University of Technology.")
    
    def launch_nodes(self):
        print("Starting launch sequence.")

        #region Gazebo launching
        # Launching gazebo simulator with specified world
        gui = True 
        use_sim_time = True
        headless = False 
        world_name = r.get_path("ridgeback_gazebo")+'/worlds/ridgeback_race.world'

        # Creating instance of Gazebo class
        self.gzb = Gazebo(gui, use_sim_time, headless, world_name)

        # Launching the gazebo simulator
        self.gzb.launch(uuid)

        # Running the map server on the existing gzb.launch object
        map_file = r.get_path("multi_ridgeback_nav") + '/maps/my_ridgeback_race.yaml'
        map_server_node = roslaunch.core.Node('map_server', 'map_server', name='map_server', args=map_file)
        self.gzb.launch.launch(map_server_node)
        #endregion

        #region Robot launching
        # Loop through robot list and spawn the robots
        for robot in self.launch_list:
            robot.launch(uuid, amcl = True, move_base = True, sfm_mpdm = False)
        #endregion

    def shutdown_nodes(self):
        print("Starting shutdown sequence.")
        self.gzb.launch.parent.shutdown()   # Shut down Gazebo
        for robot in self.launch_list:      # Shut down robots
            robot.shutdown()

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('multi_robot_sim_launcher', anonymous=False)
    
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # creating instance of ROSPack class to use for path resolution to ros packages
        r = rospkg.RosPack()

        #region --- GUI ---
        app = QtGui.QApplication(sys.argv)
        appGui = GuiMainWindow()
        windowIcon = QtGui.QIcon()
        windowIcon.addPixmap(QtGui.QPixmap(":/icons/Next.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        appGui.setWindowIcon(windowIcon)
        appGui.show()
        app.exec_()
        #endregion
        

        # Shut down launched nodes (if the launch_status is launched of course)
        if launch_status == launchStatus.LAUNCHED:
            appGui.shutdown_nodes()
            print "Closing shutdown sequence complete!"
        else:
            print("No launched nodes to shutdown; exiting.")
    except rospy.ROSInterruptException:
        pass