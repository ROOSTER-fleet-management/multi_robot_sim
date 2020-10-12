
// The main() just initializes ROS, creates a QApplication, 
// creates the top-level widget (of type "DynaRviz"), 
// shows it, and runs the Qt event loop.

#include <QApplication>
#include <ros/ros.h>
#include "dynaRviz.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "dynaRviz", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  DynaRviz* dynaRviz = new DynaRviz();
  dynaRviz->show();

  app.exec();

  delete dynaRviz;
}
