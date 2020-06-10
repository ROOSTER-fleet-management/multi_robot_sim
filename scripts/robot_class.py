
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


        
