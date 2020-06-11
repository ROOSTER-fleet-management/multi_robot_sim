#!/home/melodic/virtual_environments/roslaunch_api_2/bin/python

#/usr/bin/env python

import rospy
import roslaunch
import rospkg
from robot_class import Robot

#from gazebo_msgs import DeleteModel

def launcher():
    rospy.init_node('multi_robot_sim_launcher', anonymous=False)
   
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    #print "uuid=", uuid

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

    #running the map server on the existing launch_gzb object
    map_file = r.get_path("multi_ridgeback_nav") + '/maps/my_ridgeback_race.yaml'
    map_server_node = roslaunch.core.Node('map_server', 'map_server', name='map_server', args=map_file)
    launch_gzb.launch(map_server_node)

    #spawning the robots
    rdg01 = Robot('rdg01',7.2, -2.2, 0)

    active_robots = [rdg01] #list of instances of class Robot that are in service

        robot.launch(uuid, amcl = True, move_base = True, sfm_mpdm = False)

        launch_rdg_args = launch_rdg_cmd[1:]
        launch_rdg_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_rdg_cmd)[0],launch_rdg_args)]
        launch_rdg = roslaunch.scriptapi.ROSLaunch()
        launch_rdg.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_rdg_file)
        #Launching the robot spawning and navigation stack nodes
        launch_rdg.start()
               
    rospy.spin()
    
    rdg01.shutdown()
    rdg02.shutdown()
    print "shut Down sequence complete!"



if __name__ == '__main__':
    try:
        launcher()
    except rospy.ROSInterruptException:
        pass




    