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
#include <sstream>

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

  //reading the robot_list from the parameter server into an std::string
  std::string robotListString;
  ros::param::get("/robot_list",robotListString);

  //Extracting substrings into a vector of strings
  std::stringstream robotListStringStream( robotListString );
  std::vector<std::string> robotListStringVector;
  while( robotListStringStream.good() )
    {
      std::string substr;
      getline( robotListStringStream, substr, ',' );
      robotListStringVector.push_back( substr );
    }
  robotListStringVector.resize(robotListStringVector.size()-1);
  robotListStringVector[0].erase(robotListStringVector[0].begin());

  //ROS_INFO("size of robotlistvector = %lu",robotListStringVector.size());
  // for(std::size_t i = 0; i < robotListStringVector.size(); i++)
  // {
  //   ROS_INFO("%s",robotListStringVector[i].data());
  // }
  
  
  //obtaining the rootDisplayGroup from the visualization manager
  rviz::DisplayGroup* rootDisplayGroup_  = manager_->getRootDisplayGroup();
  
  //creating a vector of RobotDisplayGroups for each robot in the robot_list and adding the RobotDisplayGroups to the rootDisplayGroup
  std::vector<RobotDisplayGroup*> RobotDisplayGroupVector;
  RobotDisplayGroupVector.resize(robotListStringVector.size());
  for (std::string robotId : robotListStringVector)
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

