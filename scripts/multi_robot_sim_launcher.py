#!/usr/bin/env python

import rospy
import roslaunch
import rospkg
from robot_class import Robot
import sys
from PyQt4 import QtGui

#from gazebo_msgs import DeleteModel

class Window(QtGui.QMainWindow):
    def __init__(self):
        super(Window, self).__init__()

        '''# Set up launch api objects
        self.launch_gzb = roslaunch.scriptapi.ROSLaunch()
        self.launch_rdg = roslaunch.scriptapi.ROSLaunch()
        '''

        # Set up and launch Gui window
        w = QtGui.QWidget()
        b = QtGui.QLabel(w)
        b.setText("Hello World!")
        w.setGeometry(500,300,500,300)
        b.move(50,20)
        w.setWindowTitle("Multi robot fleet launcher")
        w.show()
    
    '''
    def launch(self):
        # launching gazebo simulator with specified world
        use_sim_time = True
        gui = True 
        headless = False 
        r = rospkg.RosPack()
        world_name = r.get_path("ridgeback_gazebo")+'/worlds/ridgeback_race.world'  

        launch_gzb_cmd = [r.get_path("gazebo_ros")+'/launch/empty_world.launch', 
                        'debug:=0', 
                        'gui:=' + str(gui),
                        'use_sim_time:=' + str(use_sim_time),
                        'headless:=' + str(headless),
                        'world_name:=' + world_name,
                        'paused:=false'
                        ]

        launch_gzb_args = launch_gzb_cmd[1:]
        launch_gzb_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_gzb_cmd)[0],launch_gzb_args)]
        # launch_gzb = roslaunch.scriptapi.ROSLaunch()
        self.launch_gzb.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_gzb_file)
        #Launching the gazebo world only
        self.launch_gzb.start()

        #running the map server on the existing launch_gzb object
        map_file = r.get_path("multi_ridgeback_nav") + '/maps/my_ridgeback_race.yaml'
        map_server_node = roslaunch.core.Node('map_server', 'map_server', name='map_server', args=map_file)
        self.launch_gzb.launch(map_server_node)

        #spawning the robots
        rdg01 = Robot('rdg01',7.2, -2.2, 0)

        active_robots = [rdg01] #list of instances of class Robot that are in service

        for robot_id in active_robots:
            launch_rdg_cmd = [r.get_path("multi_ridgeback_nav")+'/launch/include/one_robot.launch', 
                        'namespace:=' + robot_id.namespace ,
                        'tfpre:='+ robot_id.tf_prefix ,
                        'robot_name:=' + robot_id.name ,
                        'initX:=' + str(robot_id.init_x) , 
                        'initY:=' + str(robot_id.init_y) ,
                        'initYaw:=' + str(robot_id.init_yaw), 
                        'scanTopic:=' + robot_id.scan_topic ,
                        'baseFrame:=' + robot_id.base_frame ,
                        'odomFrame:=' + robot_id.odom_frame ,
                        ]          

            launch_rdg_args = launch_rdg_cmd[1:]
            launch_rdg_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_rdg_cmd)[0],launch_rdg_args)]
            # launch_rdg = roslaunch.scriptapi.ROSLaunch()
            self.launch_rdg.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_rdg_file)
            #Launching the robot spawning and navigation stack nodes
            self.launch_rdg.start()
    '''


def launcher():
    '''
    # Initialize the node
    rospy.init_node('multi_robot_sim_launcher', anonymous=False)
   
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    #print "uuid=", uuid
    '''

 

    '''
    gui.launch_gzb.parent.shutdown()
    gui.launch_rdg.parent.shutdown()
    print "shut Down sequence complete!"
    '''


if __name__ == '__main__':
    # try:
    # Initialize Qt
    app = QtGui.QApplication(sys.argv)

    # Set up the GUI
    gui = Window()

    # Run the GUI application loop
    sys.exit(app.exec_())

    # launcher()
    # except rospy.ROSInterruptException:
    #    pass