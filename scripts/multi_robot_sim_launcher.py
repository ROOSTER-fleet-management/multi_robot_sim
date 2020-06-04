#!/usr/bin/env python

import rospy
import roslaunch
import rospkg

#from gazebo_msgs import DeleteModel

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
    
    launch2 = [r.get_path("multi_ridgeback_gazebo")+'/launch/include/one_robot.launch', 
                'namespace:=' + namespace ,
                'tfpre:='+ tfpre ,
                'robot_name:=' + robot_name ,
                'initX:=' + str(initX) , 
                'initY:=' + str(initY) ,
                'initYaw:=' + str(initYaw) ,
              ]

    launch2_args = launch2[1:]
    launch2_file = roslaunch.rlutil.resolve_launch_arguments(launch2)[0]
    #launch2_file = [(roslaunch.rlutil.resolve_launch_arguments(launch2)[0],launch2_args)]
    #parent2 = roslaunch.parent.ROSLaunchParent(uuid, launch2_file)
    #parent2.start()

    launch_files = [(launch1_file, launch1_args), (launch2_file, launch2_args)]
    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    parent.start()

    # a = raw_input("shutdown key please. r for killing ridgeback nodes only")

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