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

class RobotDisplayGroup : public rviz::DisplayGroup
{
  private:
    std::string robotId_;

  public:
    RobotDisplayGroup(std::string robotId, rviz::VisualizationManager* manager_)
      : robotId_(robotId)
      {
        this->setName(robotId_.data());
        this->initialize(manager_);
        this->setEnabled(1);

        //creating Laser Scan
        std::string scanTopic = "/"+robotId+"/front/scan";
        rviz::Display* laserScan_ = this->createDisplay("rviz/LaserScan"); 
        this->addDisplay(laserScan_);
        laserScan_->initialize(manager_);
        laserScan_->setEnabled(1);
        laserScan_->setName("Laser Scan");
        laserScan_->subProp( "Topic" )->setValue( scanTopic.data() );
      }

};


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
  // grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
  // ROS_ASSERT( grid_ != NULL );

  // // Configure the GridDisplay the way we like it.
  // grid_->subProp( "Line Style" )->setValue( "Billboards" );
  // grid_->subProp( "Color" )->setValue( QColor( Qt::yellow ) );

  //reading the robot_list from the parameter server
  std::string robot_list;
  ros::param::get("/robot_list",robot_list);
  ROS_INFO("%s",robot_list.data());

  rviz::DisplayGroup* root_disp_group_  = manager_->getRootDisplayGroup();
  // int rootChildNum = root_disp_group_->numChildren();
  // ROS_INFO("%i",rootChildNum);
  // rviz::Display* dispTemp = root_disp_group_->getDisplayAt(0);
  // ROS_INFO("%s",dispTemp->getNameStd().data());
  // ROS_INFO("%s",root_disp_group_->getDisplayAt(1)->getNameStd().data());
  viz_frame_->setStatus("oooolalalala"); //sets the status message displayed in the status bar at the bottom

  //creating a new RobotDisplayGroup
  RobotDisplayGroup* rdg01 = new RobotDisplayGroup(std::string("rdg01"), manager_);

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



//   rviz::PropertyTreeModel* dispTree = manager_->getDisplayTreeModel();
//   rviz::Property* root_prop = dispTree->getRoot();
//   ROS_INFO("%s",root_prop->getNameStd().data());
//   //dispTree->beginInsert(rdg01, rviz::Display::numChildren);