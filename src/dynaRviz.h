#ifndef DYNARVIZ_H
#define DYNARVIZ_H

#include <QWidget>

namespace rviz
{
class VisualizationManager;
class VisualizationFrame;
class DisplayGroup;
}

// Class "DynaRviz" implements the top level widget for this example.
class DynaRviz: public QWidget
{
Q_OBJECT
public:
  DynaRviz( QWidget* parent = 0 );
  virtual ~DynaRviz();

private:
  rviz::VisualizationFrame* vizFrame_;
  rviz::VisualizationManager* manager_;
  rviz::DisplayGroup* rootDisplayGroup_;
};

#endif // DYNARVIZ_H
