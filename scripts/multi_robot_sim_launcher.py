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

    launch1 = [r.get_path("gazebo_ros")+'/launch/empty_world.launch', 
             'debug:=0', 
             'gui:=' + str(gui),
             'use_sim_time:=' + str(use_sim_time),
             'headless:=' + str(headless),
             'world_name:=' + world_name,
             'paused:=false'
             ]

    launch1_args = launch1[1:]
    launch1_file = roslaunch.rlutil.resolve_launch_arguments(launch1)[0]
    
    #launch1_file = [(roslaunch.rlutil.resolve_launch_arguments(launch1)[0],launch1_args)]
    #parent1 = roslaunch.parent.ROSLaunchParent(uuid, launch1_file)
    #parent1.start()

    #spawning the first robot
    robot_type = 'rdg'
    robot_number = '01'
    robot_id = robot_type + robot_number
    robot_name = 'ridgeback_' + robot_id
    namespace = robot_id
    tfpre = namespace + '_tf'
    initX = 7.2 
    initY = -2.2 
    initYaw = 0
    
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

    # if a=='r':
    #     parent2.shutdown()
        
    rospy.spin()

    #parent1.shutdown()
    #parent2.shutdown()
    parent.shutdown()
    print "shut Down sequence complete!"

    # rospy.on_shutdown(shutDownHook)
        
# def shutDownHook():
#   print "shutdown time!"
#   parent.shutdown()


if __name__ == '__main__':
    try:
        launcher()
    except rospy.ROSInterruptException:
        pass