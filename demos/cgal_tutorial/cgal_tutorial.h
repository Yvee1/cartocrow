//
// Created by steven on 12/22/23.
//

#ifndef CARTOCROW_CGAL_TUTORIAL_H
#define CARTOCROW_CGAL_TUTORIAL_H

#include <QMainWindow>
#include "cartocrow/core/core.h"
#include <cartocrow/renderer/geometry_widget.h>

using namespace cartocrow;
using namespace cartocrow::renderer;

class CGALTutorial : public QMainWindow {
	Q_OBJECT

  public:
	CGALTutorial();
	void recalculate();

  private:
	std::vector<std::shared_ptr<Point<Inexact>>> m_points;
	std::vector<Point<Inexact>> m_result;
	GeometryWidget* m_renderer;
};

class ConvexHullPainting : public GeometryPainting {
  public:
	ConvexHullPainting(std::shared_ptr<std::vector<Point<Inexact>>> points,
	                   std::shared_ptr<Polygon<Inexact>> convex_hull);

  protected:
	void paint(GeometryRenderer& renderer) const override;

  private:
	std::shared_ptr<Polygon<Inexact>> m_convex_hull;
	std::shared_ptr<std::vector<Point<Inexact>>> m_points;
};

#endif //CARTOCROW_CGAL_TUTORIAL_H
