import rospkg
import roslaunch
r = rospkg.RosPack()

class Gazebo:
    
    def __init__(self, gui, use_sim_time, headless, world_name):
        self.gui = gui
        self.use_sim_time = use_sim_time
        self.headless = headless
        self.world_name = world_name
        self.paused = False

    def launch(self,uuid):
        launch_cmd = [r.get_path("gazebo_ros")+'/launch/empty_world.launch', 
                        'debug:=0', 
                        'gui:=' + str(self.gui),
                        'use_sim_time:=' + str(self.use_sim_time),
                        'headless:=' + str(self.headless),
                        'world_name:=' + self.world_name,
                        'paused:=' + str(self.paused)
                        ]
        
        launch_args = launch_cmd[1:]
        launch_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_cmd)[0],launch_args)]
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_file)
        #Launching the gazebo world only
        self.launch.start()
