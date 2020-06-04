#!/usr/bin/env python

import rospy
import roslaunch
import rospkg

#from gazebo_msgs import DeleteModel


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



def launcher():
    rospy.init_node('multi_robot_sim_launcher', anonymous=False)
   
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    print "uuid=", uuid

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
    launch_gzb = roslaunch.scriptapi.ROSLaunch()
    launch_gzb.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_gzb_file)
    #Launching the gazebo world only
    launch_gzb.start()

    #running the map server
    map_file = r.get_path("multi_ridgeback_nav") + '/maps/my_ridgeback_race.yaml'
    print map_file
    map_server_node = roslaunch.core.Node('map_server', 'map_server', name='map_server', args=map_file)

    launch_gzb.launch(map_server_node)

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
        launch_rdg = roslaunch.scriptapi.ROSLaunch()
        launch_rdg.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_rdg_file)
        #Launching the robot spawning and navigation stack nodes
        launch_rdg.start()
               
    rospy.spin()
    
    launch_gzb.shutdown()
    launch_rdg.shutdown()
    print "shut Down sequence complete!"


if __name__ == '__main__':
    try:
        launcher()
    except rospy.ROSInterruptException:
        pass




    