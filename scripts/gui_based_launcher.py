#!/usr/bin/env python

import rospy
import roslaunch
import rospkg
from robot_class import Robot
import sys
from PyQt4 import QtGui, QtCore
from ui import gui_launcher_node


################## TODOLIST ##################################
# TODO 1. Add question ' are you sure '  on shutdown.
# TODO 2. Add question prompt ' Are you sure'  on Quit application
# TODO 3. Add about popup information
# TODO 4. Add right mouse button delete items from launch tree
# TODO 5. Add launch tree item adding. 
# TODO 6. Change launch status labeltext and labelicon based on launch_status variable.
# TODO 7. Add clearing everything when clicking the 'New' button in the File menu with question prompt if they are sure.
# TODO 8. Add saving and loading of docking station variables based on comboBox robot type.
# TODO 9. Add saving and loading of the launch tree
# TODO 10. Combine saving and loading of launch tree and all docking station variables to the File > Save and File > Load menu, allowing to save and load to and from a file.
###############################################################


print("Test04")

class launchStatus:             # Kind of like an enum but for python 2.7
    AWAITING = "Awaiting launch..."
    LAUNCHED = "Launched!"
    SHUTDOWN = "Shutdown."

launch_status = launchStatus.AWAITING

class GuiMainWindow(gui_launcher_node.Ui_MainWindow, QtGui.QMainWindow):
    def __init__(self):
        super(GuiMainWindow, self).__init__()
        self.setupUi(self)
        self.btnClearLaunchList.clicked.connect(self.clear_launch_tree)
        self.btnLaunch.clicked.connect(self.launch)
        self.btnShutdown.clicked.connect(self.shutdown)

    def clear_launch_tree(self):
        self.robotTree.clear()
    
    def launch(self):
        launch_list = ['rdg01']    # Iterate over items in tree widget with QTreeWidgetItemIterator ?
        if not launch_list:
            # launch_list is empty, throw error.
            QtGui.QMessageBox.warning(self, "Launch list is empty!", "You must have at least 1 robot in the 'Launch list' before you can launch.")
        else:
            global launch_status
            launch_status = launchStatus.LAUNCHED
            print("Launching!")
            print(launch_status)

    def shutdown(self):
        if launch_status != launchStatus.LAUNCHED:
            # It was not launched yet, thus cannot shutdown.
            QtGui.QMessageBox.about(self, "Nothing to shutdown.", "No launch was found to shutdown.")
        else:
            # TODO: Ask a question if the user is sure to shutdown?
            global launch_status
            launch_status = launchStatus.SHUTDOWN
            print("Shutting down!")
            print(launch_status)


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
        appGui = GuiMainWindow()
        appGui.show()
        app.exec_()
        #endregion
        

        # Shut down launched nodes
        launch_gzb.parent.shutdown()
        launch_rdg.parent.shutdown()
        print "shut Down sequence complete!"
    except rospy.ROSInterruptException:
        pass