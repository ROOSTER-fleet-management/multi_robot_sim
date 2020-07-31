#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

#include "rviz/visualization_frame.h"
#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/display_group.h"
#include "rviz/properties/property_tree_model.h"

#include "dynaRviz.h"
#include <string>
#include <unistd.h>

// BEGIN_TUTORIAL
// Constructor for DynaRviz.  This does most of the work of the class.
DynaRviz::DynaRviz( QWidget* parent )
  : QWidget( parent )
{
  viz_frame_ = new rviz::VisualizationFrame();
  viz_frame_->initialize();
  viz_frame_->setSplashPath("");
  
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( viz_frame_ );
  manager_ = viz_frame_->getManager();

  // Set the top-level layout for this DynaRviz widget.
  setLayout( main_layout );

  /////////////////////////////////////////////////////////////////////////////////////////
    // Create a Grid display.
  grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
  ROS_ASSERT( grid_ != NULL );

  // Configure the GridDisplay the way we like it.
  grid_->subProp( "Line Style" )->setValue( "Billboards" );
  grid_->subProp( "Color" )->setValue( QColor( Qt::yellow ) );

/////////////////////////////////////////////////////////reading robot_list from param_server /////////////////////////
  // bool is_robot_list, use_sim_time;
  // is_robot_list = ros::param::has("/robot_list") ; //ros::param::has("/robot_list");
  // ROS_INFO("%i",is_robot_list);
  
  // std::string robot_list;
  // //ros::Rate r(1); // 1 hz
  // // while (!ros::param::get("/robot_list",robot_list))
  // // {
  // //   //ros::spinOnce();
  // //   r.sleep();
  // // }
  // ros::param::get("/robot_list",robot_list);
  // ROS_INFO("%s",robot_list.data());

  // sleep(2);
  
  // while(robot_list.empty())
  // {
  //   ros::param::get("/robot_list",robot_list);
  //   ROS_INFO("%s",robot_list.data());
  //   sleep(1);
  // }
  // ROS_INFO("%s",robot_list.data());

////////////////////////////////////////////////////////////////////////////////////////////////////
  //reading the robot_list from the parameter server
  std::string robot_list;
  ros::param::get("/robot_list",robot_list);
  ROS_INFO("%s",robot_list.data());

  rviz::DisplayGroup* root_disp_group_  = manager_->getRootDisplayGroup();
  int rootChildNum = root_disp_group_->numChildren();
  ROS_INFO("%i",rootChildNum);
  rviz::Display* dispTemp = root_disp_group_->getDisplayAt(0);
  ROS_INFO("%s",dispTemp->getNameStd().data());
  ROS_INFO("%s",root_disp_group_->getDisplayAt(1)->getNameStd().data());
  viz_frame_->setStatus("oooolalalala"); //sets the status message displayed in the status bar at the bottom

  rviz::DisplayGroup* rdg01 = new rviz::DisplayGroup();
  rdg01->setName("rdg01");
  rdg01->initialize(manager_);
  rdg01->setEnabled(1);
  rviz::PropertyTreeModel* dispTree = manager_->getDisplayTreeModel();
  rviz::Property* root_prop = dispTree->getRoot();
  ROS_INFO("%s",root_prop->getNameStd().data());
  //dispTree->beginInsert(rdg01, rviz::Display::numChildren);
  
  rviz::Display* scan_ = rdg01->createDisplay("rviz/LaserScan"); 
  rdg01->addDisplay(scan_);
  scan_->initialize(manager_);
  scan_->setEnabled(1);
  scan_->setName("Laser Scan");
  scan_->subProp( "Topic" )->setValue( "/rdg01/front/scan" );

  root_disp_group_->addChild(rdg01);


  /////////////////////////////////////////////////////////////////////////////////////////
}

// Destructor.
DynaRviz::~DynaRviz()
{
  delete viz_frame_;
}

void DynaRviz::setThickness( int thickness_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( thickness_percent / 100.0f );
  }
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the cell size of the grid by changing the grid's
// "Cell Size" Property.
void DynaRviz::setCellSize( int cell_size_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
  }
}