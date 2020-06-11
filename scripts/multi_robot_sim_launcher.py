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
    gui = True 
    use_sim_time = True
    headless = False 
    world_name = r.get_path("ridgeback_gazebo")+'/worlds/ridgeback_race.world'  
    # creating instance of Gazebo class
    gzb = Gazebo(gui, use_sim_time, headless, world_name)
    # launching the gazebo simulator
    gzb.launch(uuid)

    #running the map server on the existing gzb.launch object
    map_file = r.get_path("multi_ridgeback_nav") + '/maps/my_ridgeback_race.yaml'
    map_server_node = roslaunch.core.Node('map_server', 'map_server', name='map_server', args=map_file)
    gzb.launch.launch(map_server_node)


    #spawning the robots
    rdg01 = Robot('rdg01')
    rdg01.assign_cell(ds01)
    
    rdg02 = Robot('rdg02')
    rdg02.assign_cell(ds01)

    # list of instances of class Robot that are in service
    active_robots = [rdg01,rdg02] 

    # spawning the robots
    for robot in active_robots:
        robot.launch(uuid, amcl = True, move_base = True, sfm_mpdm = False)

               
    rospy.spin()
    
    rdg01.shutdown()
    rdg02.shutdown()
    print "shut Down sequence complete!"



if __name__ == '__main__':
    try:
        launcher()
    except rospy.ROSInterruptException:
        pass




    