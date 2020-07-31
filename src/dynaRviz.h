#ifndef DYNARVIZ_H
#define DYNARVIZ_H

#include <QWidget>

namespace rviz
{
class Display;
class RenderPanel;
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

private Q_SLOTS:
  void setThickness( int thickness_percent );
  void setCellSize( int cell_size_percent );

private:
  rviz::VisualizationFrame* viz_frame_;
  rviz::VisualizationManager* manager_;
  rviz::DisplayGroup* root_disp_group_;
  rviz::Display* grid_;
};

#endif // DYNARVIZ_H
