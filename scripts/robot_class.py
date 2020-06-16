import rospkg
import roslaunch
import rospy

# creating instance of ROSPack class to resolve paths of ROS packages
r = rospkg.RosPack()

class Robot:
    """ This class encapsulates the data pertaining to a robot """
    def __init__(self, robot_id):
        self.id = robot_id
        self.type = robot_id[0:3]
        if self.type == 'rdg':
            self.name = 'ridgeback_' + robot_id
        
        self.namespace = robot_id   # namespace under which the nodes and topics of the robot will be launched
        self.tf_prefix = self.namespace + '_tf' # the coordinate frames of the robot will have this prefix

        self.scan_topic = 'front/scan'  
        self.base_frame = self.tf_prefix +'/base_link'
        self.odom_frame = self.tf_prefix +'/odom' 
        
    def assign_cell(self, docking_station):
        """ This method assigns the robot to a docking cell in the specifed docking station """
        self.station_id = docking_station.id
        for cell in docking_station.cell:
            if cell.assigned == False:
                self.cell_id = cell.id
                self.cell_origin = cell.origin
                cell.assigned = True
                cell.robot_type = self.type
                break
    
    def launch(self, uuid, sfm_mpdm_enabled, robot_list):
        """ This method launches the launch file of the robot """
        self.sfm_mpdm_enabled = sfm_mpdm_enabled
        self.robotlist = robot_list
        
        launch_cmd = [r.get_path("multi_ridgeback_nav")+'/launch/include/one_robot.launch', 
                    'namespace:=' + self.namespace ,
                    'tfpre:='+ self.tf_prefix ,
                    'robot_name:=' + self.name ,
                    'initX:=' + str(self.cell_origin.x) , 
                    'initY:=' + str(self.cell_origin.y) ,
                    'initYaw:=' + str(self.cell_origin.theta), 
                    'scanTopic:=' + self.scan_topic ,
                    'baseFrame:=' + self.base_frame ,
                    'odomFrame:=' + self.odom_frame ,
                    'sfm_mpdm_enabled:=' + self.sfm_mpdm_enabled,
                    'robotlist:=' + self.robotlist
                    ]  
                    
        launch_args = launch_cmd[1:]
        launch_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_cmd)[0],launch_args)]
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_file)
        #Launching the robot with specified nodes
        self.launch.start()

    def shutdown(self):
        """ This method brings down the nodes launched by the launch file """
        self.launch.parent.shutdown()


    # def return_to_cell(self):
    #     from geometry_msgs.msg import PoseStamped
    #     topic = self.id + 'move_base_simple/goal'
    #     pub = rospy.Publisher(topic, PoseStamped, queue_size=10)

        
