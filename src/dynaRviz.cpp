#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

#include "rviz/visualization_frame.h"
#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include <string>
#include <vector>

#include "dynaRviz.h"
#include "robotDisplayGroup.h"

// Constructor for DynaRviz.  This does most of the work of the class.
DynaRviz::DynaRviz( QWidget* parent )
  : QWidget( parent )
{
  vizFrame_ = new rviz::VisualizationFrame();
  vizFrame_->initialize();
  vizFrame_->setSplashPath("");
  
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( vizFrame_ );
  manager_ = vizFrame_->getManager();

  // Set the top-level layout for this DynaRviz widget.
  setLayout( main_layout );

  //reading the robot_list from the parameter server 
  std::vector<std::string> robotListVector; 
  ros::param::get("/robot_list",robotListVector);

  // printing out the robot_ids for debugging 
  // ROS_INFO("size of robotlistvector = %lu",robotListVector.size());
  // for(std::size_t i = 0; i < robotListVector.size(); i++)
  // {
  //   ROS_INFO("%s",robotListVector[i].data());
  // }
  
  //obtaining the rootDisplayGroup from the visualization manager
  rviz::DisplayGroup* rootDisplayGroup_  = manager_->getRootDisplayGroup();
  
  //Adding Map display
  std::string mapTopic_ = "/map";
  rviz::Display* map_ = manager_->createDisplay("rviz/Map","Map", 1);
  map_->subProp( "Topic" )->setValue( mapTopic_.data() );

  //Adding Marker Array display
  std::string locationArrayTopic_ = "/visualization_marker_array";
  rviz::Display* locationArray_ = manager_->createDisplay("rviz/MarkerArray","Locations", 1);
  locationArray_->subProp( "Topic" )->setValue( locationArrayTopic_.data() );

  
  //creating a vector of RobotDisplayGroups for each robot in the robot_list and adding the RobotDisplayGroups to the rootDisplayGroup
  std::vector<RobotDisplayGroup*> RobotDisplayGroupVector;
  RobotDisplayGroupVector.resize(robotListVector.size());
  for (std::string robotId : robotListVector)
  {
    RobotDisplayGroup* tempPointer = new RobotDisplayGroup(robotId.data(), manager_);
    RobotDisplayGroupVector.push_back(tempPointer);
    rootDisplayGroup_->addChild(tempPointer); 
  }      
  
  vizFrame_->setStatus("All robots spawned successfully."); //sets the status message displayed in the status bar at the bottom

}

// Destructor.
DynaRviz::~DynaRviz()
{
  delete vizFrame_;
}

