
class Robot:
    def __init__(self, robot_id, init_x, init_y, init_yaw):
        self.robot_id = robot_id
        self.type = robot_id[0:3]
        if self.type == 'rdg':
            self.name = 'ridgeback_' + robot_id
        
        self.namespace = robot_id
        self.tf_prefix = self.namespace + '_tf'
        self.init_x = init_x 
        self.init_y = init_y 
        self.init_yaw = init_yaw

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

    def launch(self, uuid, amcl = True, move_base = True, sfm_mpdm = True):
        """ This method launches the launch file of the robot """
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
                    ]  
                    
        launch_args = launch_cmd[1:]
        launch_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_cmd)[0],launch_args)]
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_file)
        #Launching the robot with specified nodes
        self.launch.start()
        
