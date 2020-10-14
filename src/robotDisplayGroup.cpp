#include "robotDisplayGroup.h"

RobotDisplayGroup::RobotDisplayGroup(std::string robotId, rviz::VisualizationManager* manager_)
    : robotId_(robotId)
{
    this->setName(robotId_.data());
    this->initialize(manager_);
    this->setEnabled(1);

    //creating RobotModel display
    modelParameter_ = "/"+robotId_+"/robot_description";
    robotModel_ = this->createDisplay("rviz/RobotModel"); 
    this->addDisplay(robotModel_);
    robotModel_->initialize(manager_);
    robotModel_->setEnabled(1);
    robotModel_->setName("Robot Model");
    robotModel_->subProp( "Robot Description" )->setValue( modelParameter_.data() );
    
    //Creating robot footprint display
    footprintTopic_ = "/"+robotId_+"/move_base/local_costmap/footprint";
    robotFootprint_ = this->createDisplay("rviz/Polygon");
    this->addDisplay(robotFootprint_);
    robotFootprint_->initialize(manager_);
    robotFootprint_->setEnabled(1);
    robotFootprint_->setName("Robot Footprint");
    robotFootprint_->subProp( "Topic" )->setValue( footprintTopic_.data() );
    
    //Creating base_link Frame axes display
    baselinkFrame_ = robotId_+"_tf/base_link";
    baselinkAxes_ = this->createDisplay("rviz/Axes");
    this->addDisplay(baselinkAxes_);
    baselinkAxes_->initialize(manager_);
    baselinkAxes_->setEnabled(1);
    baselinkAxes_->setName("base_link Frame");
    baselinkAxes_->subProp( "Reference Frame" )->setValue( baselinkFrame_.data() );

    //Creating odom Frame axes display
    odomFrame_ = robotId_+"_tf/odom";
    odomAxes_ = this->createDisplay("rviz/Axes");
    this->addDisplay(odomAxes_);
    odomAxes_->initialize(manager_);
    odomAxes_->setEnabled(1);
    odomAxes_->setName("odom Frame");
    odomAxes_->subProp( "Reference Frame" )->setValue( odomFrame_.data() );

    //creating Laser Scan display
    scanTopic_ = "/"+robotId_+"/front/scan";
    laserScan_ = this->createDisplay("rviz/LaserScan"); 
    this->addDisplay(laserScan_);
    laserScan_->initialize(manager_);
    laserScan_->setEnabled(1);
    laserScan_->setName("Laser Scan");
    laserScan_->subProp( "Topic" )->setValue( scanTopic_.data() );

    //Creating AMCL Robot Pose Array display
    poseTopic_ = "/"+robotId_+"/particlecloud";
    poseArray_ = this->createDisplay("rviz/PoseArray"); 
    this->addDisplay(poseArray_);
    poseArray_->initialize(manager_);
    poseArray_->setEnabled(1);
    poseArray_->setName("AMCL Pose Array");
    poseArray_->subProp( "Topic" )->setValue( poseTopic_.data() );


    //Creating Goal Pose display
    goalTopic_ = "/"+robotId_+"/move_base_simple/goal";
    goalPose_ = this->createDisplay("rviz/Pose"); 
    this->addDisplay(goalPose_);
    goalPose_->initialize(manager_);
    goalPose_->setEnabled(1);
    goalPose_->setName("Goal Pose");
    goalPose_->subProp( "Topic" )->setValue( goalTopic_.data() );

    //Creating Global Path plan display
    globalPathTopic_ = "/"+robotId_+"/move_base/NavfnROS/plan";
    globalPath_ = this->createDisplay("rviz/Path"); 
    this->addDisplay(globalPath_);
    globalPath_->initialize(manager_);
    globalPath_->setEnabled(1);
    globalPath_->setName("Global Path");
    globalPath_->subProp( "Topic" )->setValue( globalPathTopic_.data() );

    // Creating local path plan display
    localPathTopic_ = "/"+robotId_+"/move_base/TrajectoryPlannerROS/local_plan";
    localPath_ = this->createDisplay("rviz/Path"); 
    this->addDisplay(localPath_);
    localPath_->initialize(manager_);
    localPath_->setEnabled(1);
    localPath_->setName("Local Path");
    localPath_->subProp( "Topic" )->setValue( localPathTopic_.data() );

    //Creating global costmap display
    globalCostmapTopic_ = "/"+robotId_+"/move_base/global_costmap/costmap";
    globalCostmap_ = this->createDisplay("rviz/Map"); 
    this->addDisplay(globalCostmap_);
    globalCostmap_->initialize(manager_);
    globalCostmap_->setEnabled(1);
    globalCostmap_->setName("Global Costmap");
    globalCostmap_->subProp( "Topic" )->setValue( globalCostmapTopic_.data() );
    globalCostmap_->subProp( "Color Scheme" )->setValue( "costmap" );

    //creating local costmap display
    localCostmapTopic_ = "/"+robotId_+"/move_base/local_costmap/costmap";
    localCostmap_ = this->createDisplay("rviz/Map"); 
    this->addDisplay(localCostmap_);
    localCostmap_->initialize(manager_);
    localCostmap_->setEnabled(1);
    localCostmap_->setName("Local Costmap");
    localCostmap_->subProp( "Topic" )->setValue( localCostmapTopic_.data() );
    localCostmap_->subProp( "Color Scheme" )->setValue( "costmap" );

    //creating interactive twist command marker display
    interactiveTwistMarkerTopic_ = "/"+robotId_+"/twist_marker_server/update";
    interactiveTwistMarker_ = this->createDisplay("rviz/InteractiveMarkers"); 
    this->addDisplay(interactiveTwistMarker_);
    interactiveTwistMarker_->initialize(manager_);
    interactiveTwistMarker_->setEnabled(1);
    interactiveTwistMarker_->setName("Teleoperation");
    interactiveTwistMarker_->subProp("Update Topic")->setValue( interactiveTwistMarkerTopic_.data() );
    interactiveTwistMarker_->subProp("Show Axes")->setValue( "false" );
    interactiveTwistMarker_->subProp("Show Descriptions")->setValue( "false" );
    interactiveTwistMarker_->subProp("Show Visual Aids")->setValue( "true" );
    interactiveTwistMarker_->subProp("Enable Transparency")->setValue( "false" );
}  
   
