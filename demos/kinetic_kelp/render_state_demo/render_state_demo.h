#ifndef CARTOCROW_RENDER_STATE_DEMO_H
#define CARTOCROW_RENDER_STATE_DEMO_H

#include <QMainWindow>

#include "cartocrow/core/core.h"
#include "cartocrow/renderer/geometry_widget.h"

#include "cartocrow/kinetic_kelp/cat_point.h"

using namespace cartocrow;
using namespace cartocrow::renderer;
using namespace cartocrow::kinetic_kelp;

class RenderStateDemo: public QMainWindow {
    Q_OBJECT

  public:
    RenderStateDemo();
	void recalculate();
  private:
	std::vector<CatPoint> m_initialCatPoints;
	std::vector<std::shared_ptr<Point<Inexact>>> m_pointPointers;
	GeometryWidget* m_renderer;
};

#endif //CARTOCROW_RENDER_STATE_DEMO_H
