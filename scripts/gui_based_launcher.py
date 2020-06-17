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
# DONE 8. Add saving and loading of docking station variables based on comboBox robot type.
# TODO 9. Add saving and loading of the launch tree
# TODO 10. Combine saving and loading of launch tree and all docking station variables to the File > Save and File > Load menu, allowing to save and load to and from a file.
# DONE 11. Add an icon to the application.
# TODO 12. Add status tips where needed.
# TODO 13. Make a nice icon for the application (256x256 png)
# DONE 14. Add check to make sure clicking launch while already launched won't work. Disable button?
# DONE 15. Disable items in robotTree, add robot button and clear list button when launched.
# DONE 16. Upon launch iterate over the launch list and instantiate robot class objects.
# DONE 17. Implement "multi_robot_sim_launcher.py" functionality inside this script
# DONE 17. a) Add docking station dictionairy
# DONE 17. b) Attach robot_class.launch method to the right GUI elements.
# TODO 18. Add check for changes made to the current setup (change in docking station or change in launch list) with 'unsaved changed' flag.
# TODO 19. Add * to window title when unsaved changes are present ('unsaved changes' flag is true)
# TODO 20. Reset 'unsaved changes' flag upon save.
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
        # Set up launch_list
        self.launch_list = []

        # Set up docking stations objects
        # TODO: FIX THESE HARDCODED LOCATIONS TO SOMETHING MORE SUITABLE
        self.docking_station_dict = {
            "ds01": DockingStation('ds01',Pose2d(9.0, -5.5, math.pi/2.0)),
            "ds02": DockingStation('ds02',Pose2d(4.0, 0.5, math.pi/1.0)),
            "ds03": DockingStation('ds03',Pose2d(1.0, 5.5, math.pi*1.5))
        }

        # Set up gui
        self.setupUi(self)

        #region LAUNCH TAB
        # Set up and connect gui widgets
        self.labelStatusIcon.setPixmap(QtGui.QPixmap(":/icons/How-to.png"))
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

        # Set statustips

        #endregion

        #region DOCKING STATION TAB
        # Connect gui widgets
        self.comboBoxDockingStationID.currentIndexChanged.connect(self.docking_station_clicked)
        self.spinBoxRobotsPerColumn.valueChanged.connect(self.docking_station_value_changed)
        self.spinBoxRobotsPerRow.valueChanged.connect(self.docking_station_value_changed)
        self.spinBoxDockingStationOriginTheta.valueChanged.connect(self.docking_station_value_changed)
        self.spinBoxRobotOffsetSpacingTheta.valueChanged.connect(self.docking_station_value_changed)
        self.doubleSpinBoxDockingStationOriginX.valueChanged.connect(self.docking_station_value_changed)
        self.doubleSpinBoxDockingStationOriginY.valueChanged.connect(self.docking_station_value_changed)
        self.doubleSpinBoxRobotOffsetSpacingX.valueChanged.connect(self.docking_station_value_changed)
        self.doubleSpinBoxRobotOffsetSpacingY.valueChanged.connect(self.docking_station_value_changed)
        self.update_docking_station_gui(self.docking_station_dict[str(self.comboBoxDockingStationID.currentText())])

        # Set statustips 
        self.labelDockingStationTitle.setStatusTip("FIXME testing")
        #endregion

        #region File menu
        self.actionNew.setStatusTip("Clear current setup and start with a fresh one. Note: Any unsaved changes will be lost!")
        self.actionSave_file.setStatusTip("Save current launch and docking station setup to a file.")
        self.actionLoad_file.setStatusTip("Open an existing setup file. Note: Any unsaved changes will be lost!")
        self.actionAbout.setStatusTip("Popup window with additional information.")
        self.actionAbout.triggered.connect(self.about)
        self.actionQuit_application.setStatusTip("Quit the application. Note: Any unsaved changes will be lost!")
        self.actionQuit_application.triggered.connect(self.close_application)
        #endregion

    def docking_station_clicked(self):
        selected_ds = self.docking_station_dict[str(self.comboBoxDockingStationID.currentText())]
        self.update_docking_station_gui(selected_ds)

    def docking_station_value_changed(self):
        selected_ds = self.docking_station_dict[str(self.comboBoxDockingStationID.currentText())]
        self.update_docking_station_object(selected_ds)
    
    def update_docking_station_gui(self, ds):
        ds_id, ds_origin_x, ds_origin_y, ds_origin_theta_rad, ds_rows, ds_columns, ds_cell_offset_x, ds_cell_offset_y, ds_cell_theta_rad = ds.get_attributes()
        self.set_value_silent(self.doubleSpinBoxDockingStationOriginX, ds_origin_x)
        self.set_value_silent(self.doubleSpinBoxDockingStationOriginY, ds_origin_y)
        self.set_value_silent(self.spinBoxDockingStationOriginTheta, math.degrees(ds_origin_theta_rad))
        self.set_value_silent(self.doubleSpinBoxRobotOffsetSpacingX, ds_cell_offset_x)
        self.set_value_silent(self.doubleSpinBoxRobotOffsetSpacingY, ds_cell_offset_y)
        self.set_value_silent(self.spinBoxRobotOffsetSpacingTheta, math.degrees(ds_cell_theta_rad))
        self.set_value_silent(self.spinBoxRobotsPerRow, ds_rows)
        self.set_value_silent(self.spinBoxRobotsPerColumn, ds_columns)

    def update_docking_station_object(self, ds):
        print("changed", ds.id)
        ds_id = ds.id
        ds_origin_x = self.doubleSpinBoxDockingStationOriginX.value()
        ds_origin_y = self.doubleSpinBoxDockingStationOriginY.value()
        ds_origin_theta_deg = self.spinBoxDockingStationOriginTheta.value()
        ds_rows = self.spinBoxRobotsPerRow.value()
        ds_columns = self.spinBoxRobotsPerColumn.value()
        ds_cell_offset_x = self.doubleSpinBoxRobotOffsetSpacingX.value()
        ds_cell_offset_y = self.doubleSpinBoxRobotOffsetSpacingY.value()
        ds_cell_offset_theta_deg = self.spinBoxRobotOffsetSpacingTheta.value()
        ds_origin = Pose2d(ds_origin_x, ds_origin_y, math.radians(ds_origin_theta_deg))
        ds.set_attributes(ds_id, ds_origin, ds_rows, ds_columns, ds_cell_offset_x, ds_cell_offset_y, math.radians(ds_cell_offset_theta_deg))

    def set_value_silent(self, QtObject, value):
        QtObject.valueChanged.disconnect(self.docking_station_value_changed)
        QtObject.setValue(value)
        QtObject.valueChanged.connect(self.docking_station_value_changed)
    
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
        self.robot_navigation_dict = {}
        for i in range(child_count):
            item = root.child(i)
            robot = Robot(str(item.text(0)))
            print(self.docking_station_dict)
            print(self.docking_station_dict[str(item.text(3))])
            robot.assign_cell(self.docking_station_dict[str(item.text(3))])       # Assign docking station based on item.text(3) as key for docking station dictionairy which has all ds class objects. (ds01, ds02, ds03)
            self.robot_navigation_dict[str(item.text(0))] = str(item.text(1))
            self.launch_list.append(robot)

        if not self.launch_list:
            # launch_list is empty, throw error.
            QtGui.QMessageBox.warning(self, "Launch list is empty!", "You must have at least 1 robot in the 'Launch list' before you can launch.")
        else:
            global launch_status
            launch_status = launchStatus.LAUNCHED
            print("Launching!")
            self.labelStatusText.setText("Launching...")
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
                                            "Are you sure you want to shut down the currently launched setup? \n\nThis may take a moment...", 
                                            QtGui.QMessageBox.Yes | QtGui.QMessageBox.No):
            
            launch_status = launchStatus.SHUTDOWN
            print("Shutting down...")
            self.labelStatusText.setText("Shutting down...")
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
        # Set up the robot list (string) based on the launch_list
        robot_list_string = "["
        for robot in self.launch_list:
            robot_list_string += robot.id + ","
        robot_list_string += "]"

        # Loop through robot list and spawn the robots
        for robot in self.launch_list:
            sfm_mpdm = "True" if self.robot_navigation_dict[robot.id] == "SFM-MPDM" else "False"
            robot.launch(uuid, sfm_mpdm_enabled = sfm_mpdm, robot_list = robot_list_string)
        #endregion

    def shutdown_nodes(self):
        print("Starting shutdown sequence.")
        for robot in self.launch_list:      # Shut down robots
            robot.shutdown()
        self.gzb.launch.parent.shutdown()   # Shut down Gazebo

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