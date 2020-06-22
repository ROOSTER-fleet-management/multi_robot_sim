#!/usr/bin/env python

import rospy
import roslaunch
import rospkg   # for resolving paths to ROS packages 
import math     # for trigonometric evaluations
import json     # Used for reading and writing JSON files (saving and loading setups)
import os       # Used to get base filename and file and directory handling

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
# DONE 7. Add clearing everything when clicking the 'New' button in the File menu with question prompt if they are sure.
# DONE 8. Add saving and loading of docking station variables based on comboBox robot type.
# DONE 9. Add saving and loading of the launch tree
# DONE 10. Combine saving and loading of launch tree and all docking station variables to the File > Save and File > Load menu, allowing to save and load to and from a file.
# DONE 10a. Saving of launch tree, gui status and docking stations to JSON.
# DONE 10b. Loading of launch tree, gui status and docking stations from JSON.
# DONE 11. Add an icon to the application.
# TODO 12. Add status tips where needed.
# TODO 13. Make a nice icon for the application (256x256 png)
# DONE 14. Add check to make sure clicking launch while already launched won't work. Disable button?
# DONE 15. Disable items in robotTree, add robot button and clear list button when launched.
# DONE 16. Upon launch iterate over the launch list and instantiate robot class objects.
# DONE 17. Implement "multi_robot_sim_launcher.py" functionality inside this script
# DONE 17a. Add docking station dictionairy
# DONE 17b. Attach robot_class.launch method to the right GUI elements.
# DONE 18. Add check for changes made to the current setup (change in docking station or change in launch list) with 'unsaved changed' flag.
# DONE 19. Add * to window title when unsaved changes are present ('unsaved changes' flag is true)
# TODO 20. Reset 'unsaved changes' flag upon save[X], load[_] and new file[X].
# TODO 21. Fix/change the hardcoded docking station locations to something more suitable.
# TODO 22. Add docstrings to all methods according to PEP8.
##########################################################################

VERSION = "0.1"
APPLICATION_TITLE = "multi_robot_sim launcher"

print(APPLICATION_TITLE + ". Version: "+VERSION)

class launchStatus:             # Kind of like an enum but for python 2.7
    AWAITING = "Awaiting launch..."
    LAUNCHED = "Launched"
    SHUTDOWN = "Shutdown"

launch_status = launchStatus.AWAITING

class GuiMainWindow(gui_launcher_node.Ui_MainWindow, QtGui.QMainWindow):
    def __init__(self):
        super(GuiMainWindow, self).__init__()

        # Unsaved changes flag, true if the file has been edited since last save.
        self.unsaved_changes = False
        self.filename = "Untitled.JSON"
        self.filepath = os.getcwd()+"/"+self.filename
        print(self.filepath)
        self.setWindowTitle(self.filename + " - " + APPLICATION_TITLE)

        # Set up launch_list
        self.launch_list = []

        # Set up docking stations objects
        self.populate_docking_station_dict()

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
        self.actionNew.triggered.connect(self.new_file)
        self.actionSave_file.setStatusTip("Save current launch and docking station setup to a file.")
        self.actionSave_file.triggered.connect(self.save_file_as)
        self.actionLoad_file.setStatusTip("Open an existing setup file. Note: Any unsaved changes will be lost!")
        self.actionLoad_file.triggered.connect(self.load_file)
        self.actionAbout.setStatusTip("Popup window with additional information.")
        self.actionAbout.triggered.connect(self.about)
        self.actionQuit_application.setStatusTip("Quit the application. Note: Any unsaved changes will be lost!")
        self.actionQuit_application.triggered.connect(self.close_application)
        #endregions

        # After all changes made loading the GUI elements, reset the unsaved_changes flag.
        self.unsaved_changes_inactive()
    
    def populate_docking_station_dict(self):
        self.docking_station_dict = {
            "ds01": DockingStation('ds01',Pose2d(9.0, -5.5, math.pi/2.0)),
            "ds02": DockingStation('ds02',Pose2d(4.0, 0.5, math.pi/1.0)),
            "ds03": DockingStation('ds03',Pose2d(1.0, 5.5, math.pi*1.5))
        }
    
    def unsaved_changes_active(self):
        if not self.unsaved_changes:
            self.setWindowTitle(self.filename + "* - " + APPLICATION_TITLE)
            self.unsaved_changes = True

    def unsaved_changes_inactive(self):
        self.setWindowTitle(self.filename + " - " + APPLICATION_TITLE)
        self.unsaved_changes = False

    def docking_station_clicked(self):
        selected_ds = self.docking_station_dict[str(self.comboBoxDockingStationID.currentText())]
        self.update_docking_station_gui(selected_ds)

    def docking_station_value_changed(self):
        selected_ds = self.docking_station_dict[str(self.comboBoxDockingStationID.currentText())]
        self.update_docking_station_object(selected_ds)
    
    def update_docking_station_gui(self, ds):
        '''
        The selected docking station has changed, thus silently update all input fields to match the objects variables.
        '''
        ds_id, ds_origin_x, ds_origin_y, ds_origin_theta_rad, ds_rows, ds_columns, ds_cell_offset_x, ds_cell_offset_y, ds_cell_theta_rad = ds.get_attributes()
        self.set_value_silent(self.doubleSpinBoxDockingStationOriginX, ds_origin_x)
        self.set_value_silent(self.doubleSpinBoxDockingStationOriginY, ds_origin_y)
        self.set_value_silent(self.spinBoxDockingStationOriginTheta, math.degrees(ds_origin_theta_rad))
        self.set_value_silent(self.doubleSpinBoxRobotOffsetSpacingX, ds_cell_offset_x)
        self.set_value_silent(self.doubleSpinBoxRobotOffsetSpacingY, ds_cell_offset_y)
        self.set_value_silent(self.spinBoxRobotOffsetSpacingTheta, math.degrees(ds_cell_theta_rad))
        self.set_value_silent(self.spinBoxRobotsPerRow, ds_rows)
        self.set_value_silent(self.spinBoxRobotsPerColumn, ds_columns)
        self.unsaved_changes_active()

    def update_docking_station_object(self, ds):
        '''
        A change was made to any of the input values for the docking station. Thus update the docking station object.
        '''
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
        self.unsaved_changes_active()

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
                self.unsaved_changes_active()
            else:
                QtGui.QMessageBox.information(self, "Robot ID is not unique.", "The Robot ID you tried to add already exists in the launch list and cannot be added again.")

    def open_context_menu(self):
        self.treeMenu.exec_(QtGui.QCursor.pos())

    def delete_launch_tree_item(self, item):
        if launch_status != launchStatus.LAUNCHED:
            model_index = self.robotTree.currentIndex()
            self.robotTree.takeTopLevelItem(model_index.row())
            self.unsaved_changes_active()

    def clear_launch_tree(self):
        root = self.robotTree.invisibleRootItem()
        child_count = root.childCount()
        if child_count > 0:
            self.robotTree.clear()
            self.unsaved_changes_active()
    
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
        QtGui.QMessageBox.information(self, "About - " + APPLICATION_TITLE + ".", APPLICATION_TITLE + "\nVersion: "+VERSION+"\n\nThe ROS package multi_robot_sim is created by the Human Robot Coproduction research group at the Industrial Design Engineering faculty of the Delft University of Technology.")
    
    def new_file(self):
        '''
        This method will clear all input field and the launch list, it will also delete the docking station objects and initilise new default ones.
        '''
        if launch_status != launchStatus.LAUNCHED:
            choice = QtGui.QMessageBox.question(self, 
                                                'New file?',
                                                "Are you sure you want to clear current setup and start with a new file? Any unsaved changed will be lost!", 
                                                QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
            if choice == QtGui.QMessageBox.Yes:
                # Clear robot launch list
                self.clear_launch_tree()

                # Clear input fields to default
                self.comboBoxRobotType.setCurrentIndex(0)
                self.comboBoxDockingStationLaunch.setCurrentIndex(0)
                self.comboBoxDockingStationID.setCurrentIndex(0)
                self.spinBoxRobotID.setValue(1)
                self.checkBoxSFMMPDM.setChecked(False)

                # Clear docking station objects from dict, populate dict with new docking station objects, update GUI accordingly
                self.docking_station_dict.clear()
                self.populate_docking_station_dict()
                self.update_docking_station_gui(self.docking_station_dict[str(self.comboBoxDockingStationID.currentText())])

                self.unsaved_changes_inactive()
            else:
                pass
        else:
            QtGui.QMessageBox.warning(self, "Launch list is currently active!", "Please shut down the currently active launch list before clearing the current setup and creating a new file.")
    
    def save_file_as(self):
        '''
        This method will open the Qt save dialog, allow the user to input a save name, and save the current setup as .JSON
        '''
        filepath = str(QtGui.QFileDialog.getSaveFileName(self, 'Save File', self.filepath, "JSON (*.json)"))
        filename = os.path.basename(filepath)
        if filename and filepath:
            self.filename = filename
            self.filepath = filepath
            # -- Generate a dictionary of all items in the launch list, each item being a dictionary itself with that items all robot launch data --
            savedata_launch_tree_dict = {}
            # Iterate over all existing (top level) items (a.k.a. robots) in the robotTree and add their column values to the launch_tree_dict.
            root = self.robotTree.invisibleRootItem()
            child_count = root.childCount()
            for i in range(child_count):
                item = root.child(i)
                savedata_launch_tree_dict[unicode(item.text(0))] = {
                    "robot_ID": unicode(item.text(0)),
                    "navigation": unicode(item.text(1)),
                    "robot_type": unicode(item.text(2)),
                    "ds": unicode(item.text(3))
                }

            # -- Generate a dictionary of all docking station objects, each docking station being a dictionary itself with all ds' data. --
            savedata_docking_station_dict = {}
            for ds in self.docking_station_dict.values():
                ds_id, ds_origin_x, ds_origin_y, ds_origin_theta_rad, ds_rows, ds_columns, ds_cell_offset_x, ds_cell_offset_y, ds_cell_theta_rad = ds.get_attributes()
                savedata_docking_station_dict[ds_id] = {
                    "id": ds_id,
                    "origin_x": ds_origin_x,
                    "origin_y": ds_origin_y,
                    "origin_theta_rad": ds_origin_theta_rad,
                    "rows": ds_rows,
                    "columns": ds_columns,
                    "cell_offset_x": ds_cell_offset_x,
                    "cell_offset_y": ds_cell_offset_y,
                    "cell_offset_theta_rad": ds_cell_theta_rad
                }
            
            # -- Generate top level dictionary which contains all launch input fields, a dict of the launch list and a dict of the docking station settings. --
            savedata_dict = {
                # Launch tab
                unicode(self.comboBoxRobotType.objectName()): unicode(self.comboBoxRobotType.itemText(self.comboBoxRobotType.currentIndex())),
                unicode(self.comboBoxDockingStationLaunch.objectName()): unicode(self.comboBoxDockingStationLaunch.itemText(self.comboBoxDockingStationLaunch.currentIndex())),
                unicode(self.spinBoxRobotID.objectName()): self.spinBoxRobotID.value(),
                unicode(self.checkBoxSFMMPDM.objectName()): self.checkBoxSFMMPDM.isChecked(),
                unicode(self.robotTree.objectName()): savedata_launch_tree_dict,
                
                # Docking station tab
                unicode(self.comboBoxDockingStationID.objectName()): unicode(self.comboBoxDockingStationID.itemText(self.comboBoxDockingStationID.currentIndex())),
                "docking_stations": savedata_docking_station_dict
            }
            with open(filepath, 'w') as json_savefile:
                json.dump(savedata_dict, json_savefile, indent=4)
            
            self.unsaved_changes_inactive()

    def load_file(self):
        '''
        This method will load a setup from a JSON file, overwriting any unsaved changes of the current setup.
        '''
        if launch_status != launchStatus.LAUNCHED:
            open_dialoge = False
            if self.unsaved_changes:
                choice = QtGui.QMessageBox.question(self, 
                                                'You have unsaved changes.',
                                                "Your setup has unsaved changes. Are you sure you want to overwrite the current setup and load a setup from file? Any unsaved changed will be lost!", 
                                                QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
                if choice == QtGui.QMessageBox.Yes:
                    open_dialoge = True
                else:
                    pass
            else:
                open_dialoge = True
            
            if open_dialoge:
                filepath = str(QtGui.QFileDialog.getOpenFileName(self, 'Open File', self.filepath, "JSON (*.json)"))
                filename = os.path.basename(filepath)
                if filename and filepath:
                    self.filename = filename
                    self.filepath = filepath

                    # Load JSON file into dictionary
                    loaddata_dict = None
                    with open(filepath) as json_loadfile:
                        loaddata_dict = json.load(json_loadfile)
                    
                    # -- Loop over all keys in the dictionary and update UI input fields, launch list and docking stations accordingly --
                    for key in loaddata_dict:
                        # Launch tab
                        value = loaddata_dict[key]
                        if key == unicode(self.comboBoxRobotType.objectName()):
                            self.set_combobox_from_JSON(self.comboBoxRobotType, value)
                        elif key == unicode(self.comboBoxDockingStationLaunch.objectName()):
                            self.set_combobox_from_JSON(self.comboBoxDockingStationLaunch, value)
                        elif key == unicode(self.spinBoxRobotID.objectName()):
                            self.spinBoxRobotID.setValue(value)
                        elif key == unicode(self.checkBoxSFMMPDM.objectName()):
                            self.checkBoxSFMMPDM.setChecked(value)
                        elif key == unicode(self.robotTree.objectName()):
                            self.set_launchtree_from_JSON(self.robotTree, value)
                        
                        # Docking station tab
                        elif key == "docking_stations":
                            self.set_docking_stations_from_JSON(value)
                        elif key == unicode(self.comboBoxDockingStationID.objectName()):
                            self.set_combobox_from_JSON(self.comboBoxDockingStationID, value)
                        
                    self.unsaved_changes_inactive()
            else:
                pass


        else:
            QtGui.QMessageBox.warning(self, "Launch list is currently active!", "Please shut down the currently active launch list before overwriting the current setup and loading a setup.")
    
    #region Methods used in loading from JSON
    def set_combobox_from_JSON(self, obj, value):
        index = obj.findText(value)  # get the corresponding index for specified string in combobox

        if index == -1:  # add to list if not found
            obj.insertItems(0, [value])
            index = obj.findText(value)

        obj.setCurrentIndex(index)   # preselect a combobox value by index
    
    def set_launchtree_from_JSON(self, tree, value):
        self.clear_launch_tree()

        for key in value:
            robot_id = value[key]['robot_ID']
            navigation = value[key]['navigation']
            robot_type  = value[key]['robot_type']
            docking_station = value[key]['ds']

            robot_item = QtGui.QTreeWidgetItem([robot_id, navigation, robot_type, docking_station])
            self.robotTree.addTopLevelItem(robot_item)

    def set_docking_stations_from_JSON(self, value):
        for key in value:
            ds_id = value[key]['id']
            ds = self.docking_station_dict[ds_id]
            ds_origin_x = value[key]['origin_x']
            ds_origin_y = value[key]['origin_y']
            ds_origin_theta_rad = value[key]['origin_theta_rad']
            ds_rows = value[key]['rows']
            ds_columns = value[key]['columns']
            ds_cell_offset_x = value[key]['cell_offset_x']
            ds_cell_offset_y = value[key]['cell_offset_y']
            ds_cell_offset_theta_rad = value[key]['cell_offset_theta_rad']
            ds_origin = Pose2d(ds_origin_x, ds_origin_y, ds_origin_theta_rad)
            ds.set_attributes(ds_id, ds_origin, ds_rows, ds_columns, ds_cell_offset_x, ds_cell_offset_y, ds_cell_offset_theta_rad)
        self.docking_station_clicked()

    #endregion

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