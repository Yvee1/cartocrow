#ifndef CARTOCROW_TREEMAP_DEMO_H
#define CARTOCROW_TREEMAP_DEMO_H

#include "cartocrow/treemap/orthoconvex.h"
#include "cartocrow/core/core.h"
#include "cartocrow/core/ipe_reader.h"
#include "cartocrow/renderer/geometry_painting.h"
#include "cartocrow/renderer/geometry_widget.h"
#include <QMainWindow>

using namespace cartocrow;
using namespace cartocrow::renderer;
using namespace cartocrow::treemap;

class TreemapDemo : public QMainWindow {
	Q_OBJECT

  public:
	TreemapDemo();
	void resizeEvent(QResizeEvent *event) override;

  private:
	void create_info_box(Point<Inexact> pt, NPV node);

	GeometryWidget* m_renderer;
	QFrame* m_info_box;
	std::optional<Point<Inexact>> m_info_box_position;
	std::optional<NPV> m_selected_node;
	std::optional<Treemap> m_treemap;
	Arrangement<K>::Face_handle face_at_point(const Point<K>& point);
};

class TreemapPainting : public GeometryPainting {
  public:
	TreemapPainting(Treemap treemap);
	void paint(GeometryRenderer& renderer) const override;

  private:
	Treemap m_treemap;
};

class NodePainting : public GeometryPainting {
  public:
	NodePainting(const Treemap& treemap, const std::optional<NPV>& node);
	void paint(GeometryRenderer& renderer) const override;

  private:
	const Treemap& m_treemap;
	const std::optional<NPV>& m_node;
};

#endif //CARTOCROW_TREEMAP_DEMO_H
