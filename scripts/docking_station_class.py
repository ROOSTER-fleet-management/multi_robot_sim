from transformations import Pose2d
from transformations import transform_2d

# A Docking Station consists of many Docking Cells that may or may not be of the same type.

class DockingCell(object):
    """ class encapsulating data pertaining to a docking cell """
    def __init__(self, cell_id, origin, length=1, width=1):
        self.id = cell_id   # 4 character code of the docking cell, eg. 'dc01' 
        self.type = 'rdg'   # type of the docking cell (to determine the dimensions and ameneties) 
        self.length = length    # length of the rectangular cell
        self.width = width      # width of the rectangular cell
        self.origin = origin    # The Pose2d object depicting the centrepoint and orientation (yaw) of the cell 
                                # in the docking station frame of reference, this origin 
                                # aligns with the origin of base_link frame of the robot
        self.assigned = False   # boolean to keep track of whether the cell has been assigned or not
        self.robot_type = ''    # type of the robot assigned to this cell
        self.occupied = False   # boolean to keep track of whether the cell is currently occupied or not
        

class DockingStation:
    """ class encapsulating data pertaining to a docking station """
    def __init__(self, station_id, origin):
        self.id = station_id    # 4 character code of the docking station, eg. 'ds01'
        self.origin = origin    # a Pose2d object holding the coordinates and orientation of the 
                                # origin of the docking station frame, i.e. the bottom left corner of the rectangular station
        self.rows = 2           # number of rows (rows are parallel to the x axis of the station frame)
        self.columns = 3        # number of columns (columns are parallel to the y axis of the station frame)
        self.capacity = self.rows * self.columns    # total capacity of the station
        self.x_offset = 1.0     # distance between two columns
        self.y_offset = 1.0     # distnace between two rows
        
        self.cell_type = 'rdg'  
        self.cell_length = 1.0
        self.cell_width = 1.0

        #initializing a list of DockingCell objects in a docking station 
        self.cell = []
        # Pose of the origin of docking cells (dc) in the Docking Station (ds) frame of reference
        origin_dc_x = self.x_offset + self.cell_width/2.0  
        origin_dc_y = self.y_offset + self.cell_length/2.0
        origin_dc_theta = 0
        origin_dc = Pose2d(origin_dc_x, origin_dc_y, origin_dc_theta)
        ctr_dc = 1
        
        for r in range(self.rows):
            for c in range(self.columns):
                cell_id = 'dc' + ('0' if ctr_dc<10 else '') + str(ctr_dc)
                self.cell.append(DockingCell(cell_id, transform_2d(self.origin, origin_dc)))
                ctr_dc += 1
                origin_dc.x += self.x_offset + self.cell_width 
            origin_dc.y += self.y_offset + self.cell_length
            origin_dc.x = origin_dc_x