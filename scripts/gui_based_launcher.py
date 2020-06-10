#!/usr/bin/env python

import rospy
import roslaunch
import rospkg
from robot_class import Robot
import sys
from PyQt4 import QtGui, QtCore
from ui import gui_launcher_node

print("Test04")

class GuiMainWindow(gui_launcher_node.Ui_MainWindow, QtGui.QMainWindow):
    def __init__(self):
        super(GuiMainWindow, self).__init__()
        self.setupUi(self)

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